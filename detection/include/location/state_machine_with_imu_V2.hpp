#ifndef _STATE_MACHINEV2_H_
#define _STATE_MACHINEV2_H_


#include <lidar_correction/lidarCorrection.hpp>
#include <boost/circular_buffer.hpp>
#include <receive_stm32/stm32_utcData.h>
#include <receive_xsens/Imu_withSync.h>
#include <lidar_correction/correct_vlp_xsens.hpp>
#include <boost/shared_ptr.hpp>
#include <location/locationV2.hpp>
#include <location/state_machine_debug_record.hpp>

namespace state_machineV2 {

using namespace std;
using namespace LidarCorrection;
using namespace CorrectVlpXsens;


class state_machine_with_imu
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef boost::shared_ptr<state_machine_with_imu> Ptr;

  enum Status
  {
    normal=0,
    recover=1,
    error=2
  };

  enum ImuStatus
  {
    cover=0,
    uncover=1
  };

  state_machine_with_imu(VPointCloud::Ptr pMap ,
                float targetResolution=0.5,
                float sourceResolution=1.0,
                float fitnessMaxRange=1.0,
                bool enableSwitchLidarData=false,
                bool enablePublishGlobalDenseMap=false,
                         VelodyneCorrection::ModelType modelType = VelodyneCorrection::ModelType::VLP_16):
    m_pMapManager(new locationV2::mapManager(*pMap)),
    m_pLidarLocation(new locationV2::location(m_pMapManager->getMap(0),
                                              targetResolution,
                                              sourceResolution,
                                              fitnessMaxRange)),
    m_pCorrectionTool(new VelodyneCorrection(modelType)),
    m_pCorrectVlpXsensTool(new correct_vlp_xsens(m_pCorrectionTool)),
    m_status(Status::normal), m_imuStatus(ImuStatus::uncover),
    m_countLoop(0), m_imuVaildCount(0),
    m_newScanFlag(false),
    m_getCallBackPoseFlag(false),
    m_enableSwitchLidarData(enableSwitchLidarData),
    m_enablePublishGlobalDenseMap(enablePublishGlobalDenseMap),
    m_vGlobalPose(100), m_vGlobalDeltaPose(100), m_vHeader(100), m_vImuPoseAtLidarPoseMoment(100),
    m_recoverModeMaxCount(10), m_recoverModeCounter(0),
    m_pState_machine_debug_record(new state_machine_debug_record())

  {
    m_initCheck[0]=false;
    m_initCheck[1]=false;

  }

  void updataUtcTimeData(const receive_stm32::stm32_utcData::ConstPtr hardSyncData_ptr)
  {
    m_initCheck[1] = true;
    m_pCorrectVlpXsensTool->hardSyncHandler(hardSyncData_ptr);
  }

  void updataImuData(const receive_xsens::Imu_withSync imuData)
  {
    m_pCorrectVlpXsensTool->imuHandler(imuData);
    //current_time_imu = imuData.header.stamp;
  }


  void updataCallBackPose(Eigen::Matrix4f &pose)
  {
    m_callBackPose = pose;
    m_getCallBackPoseFlag = true;
  }

  void updataLidarHardTime(const velodyne_msgs::VelodyneScan::ConstPtr scanMsg)
  {
    //处理激光雷达数据时间，进行硬件同步校准，输出第一束激光的时间
    if(m_pCorrectVlpXsensTool->setVelodyneHardTime(scanMsg, m_LidarHardTime))
    {
    }
    else
    {
      cout << "waitting sync data!" <<endl;
    }
  }

  void updataLidarData(const velodyne_msgs::VelodyneScan::ConstPtr scanMsg)
  {
    m_pState_machine_debug_record->timer_start();
    m_pNewScan = boost::make_shared<VPointCloud>();
    bool vaildFlag;//当前帧获得的激光的旋转平移是否可用。

    updataLidarHardTime(scanMsg);

    if( m_pCorrectVlpXsensTool->VelodyneScanHandler(scanMsg, *m_pNewScan, m_imuPoseAtLidarPoseMoment, \
                                               vaildFlag, false))
    {
      if(vaildFlag)
      {
        m_imuVaildCount++;
        if(m_imuVaildCount>10)
          m_imuVaildCount = 10;
      }
      else
      {
        m_imuVaildCount = 0;
      }
      m_newScanFlag = true;
    }
    else
    {
      cout << "sync has not received " <<endl;
    }
    m_pState_machine_debug_record->time_stop(state_machine_debug_record::recordType::upackTime);
  }
  inline bool isImuVaild()
  {
    return m_imuVaildCount>1 ? true : false;
  }
  /**
   * @brief initDone
   * 等待初始化完成
   * 要有utc时间和初始pose
   * @return
   */
  bool initDone()
  {
    if(m_getCallBackPoseFlag)
    {
      m_getCallBackPoseFlag = false;
      m_pLidarLocation->setPose(m_callBackPose);

      m_vGlobalPose.push_back(m_callBackPose);//初始化第一帧
      m_vGlobalDeltaPose.push_back(Eigen::Isometry3f::Identity().matrix());//随便给一个
      pcl::PCLHeader temp;
      m_vHeader.push_back(temp);
      m_vImuPoseAtLidarPoseMoment.push_back(Eigen::Isometry3f::Identity().matrix());//随便给一个

      m_initCheck[0] = true;
    }

    return m_initCheck[0] && m_initCheck[1];


    //return false;
  }

  bool loopOnce()
  {
    bool reflashState = false;

    if(m_newScanFlag)
    {
      reflashState = true;
      m_newScanFlag = false;

      if(m_status == normal)
      {
        m_pState_machine_debug_record->timer_start();
        if(isImuVaild())
        {
          //使用上一帧imu和当前imu的delta作为预估
          m_pLidarLocation->updatePose(
                m_pNewScan,
                (*(m_vImuPoseAtLidarPoseMoment.end()-1)).inverse() * m_imuPoseAtLidarPoseMoment); 

          m_pState_machine_debug_record->time_stop(state_machine_debug_record::recordType::ndtMatchingTimeWithIMU);
        }
        else
        {
          //送入当前帧，更新位置
          m_pLidarLocation->updatePose(m_pNewScan);

          m_pState_machine_debug_record->time_stop(state_machine_debug_record::recordType::ndtMatchingTimeWithoutIMU);
        }

        float* temp = m_pLidarLocation->getDebugParam();
        m_pState_machine_debug_record->updataParam(state_machine_debug_record::recordType::ndtFinalNumIteration, temp[0]);
        m_pState_machine_debug_record->updataParam(state_machine_debug_record::recordType::ndtTransformationProbability, temp[1]);


        //匹配度衡量
        float average_lenth;
        float valid_scale;
        m_pState_machine_debug_record->timer_start();
        if(m_pLidarLocation->calc_fitness_score(average_lenth, valid_scale))
        {
          m_pState_machine_debug_record->time_stop(state_machine_debug_record::recordType::ndtCalFitnessTime);
          m_pState_machine_debug_record->updataParam(state_machine_debug_record::recordType::ndtFitnessAverageLenth, average_lenth);
          m_pState_machine_debug_record->updataParam(state_machine_debug_record::recordType::ndtFitnessValidScale, valid_scale);

          if(valid_scale<0.90)//判断条件
          {
            cout << "Go to recover mode!\n this frame: valid_scale " << valid_scale << " average_lenth " << average_lenth << endl;
            m_status = recover;
            return reflashState;
          }
        }
        else
        {
          m_pState_machine_debug_record->time_stop(state_machine_debug_record::recordType::ndtCalFitnessTime);
          m_pState_machine_debug_record->updataParam(state_machine_debug_record::recordType::ndtFitnessAverageLenth, 0.0);
          m_pState_machine_debug_record->updataParam(state_machine_debug_record::recordType::ndtFitnessValidScale, 0.0);

          cout << "calc_fitness_score error" <<endl;
          m_status = recover;
          m_recoverModeCounter = 0;
          return reflashState;
        }


        //如果定位评估认为定位无效，则不会将当前位姿放入列表中

        m_vGlobalPose.push_back(m_pLidarLocation->getnewScanPose());
        m_vGlobalDeltaPose.push_back(m_pLidarLocation->getnewScanDeltaPose());
        m_vHeader.push_back(m_pNewScan->header);
        m_vImuPoseAtLidarPoseMoment.push_back(m_imuPoseAtLidarPoseMoment);
        //完成
      }

      if(m_status == recover)
      {
        cout << "recover mode" << endl;
        m_recoverModeCounter++;
        if(m_recoverModeCounter >= m_recoverModeMaxCount)
        {
          m_status = error;
          cout << "Recover mode failed. Go to error mode!" << endl;
          return reflashState;
        }

        vector<Eigen::Matrix4f> resultPose;
        vector<float> averageLenth;
        vector<float> validScale;
        m_pLidarLocation->recoverPose(m_pNewScan, *(m_vGlobalPose.end()-1), 2, 45,
                                      resultPose, averageLenth, validScale);

//        for(unsigned int i=0;i<resultPose.size(); i++)
//        {
//          cout << i <<" averageLenth " << averageLenth[i] << " validScale " << validScale[i] <<endl;
//        }

        std::vector<float>::iterator biggest = std::max_element(std::begin(validScale), std::end(validScale));
        std::cout << "Max element is " << *biggest<< " at position " << std::distance(std::begin(validScale), biggest) << std::endl;

        if(*biggest > 0.95)//认为找到了最佳pose
        {

          m_pLidarLocation->setPose(resultPose[std::distance(std::begin(validScale), biggest)]);
          m_status = normal;
          m_vGlobalPose.push_back(m_pLidarLocation->getnewScanPose());
          m_vGlobalDeltaPose.push_back(m_pLidarLocation->getnewScanDeltaPose());
          m_vHeader.push_back(m_pNewScan->header);
          m_vImuPoseAtLidarPoseMoment.push_back(m_imuPoseAtLidarPoseMoment);
          cout << "Recover mode complete. Go to normal mode!" << endl;
          return reflashState;
        }

        //在未找到正确位置时，这儿不return，用于相应后面的m_getCallBackPoseFlag。

      }

      if(m_status == error)//因为现在recover模式太强大。。无法进入error模式，有待测试
      {
        cout << "error mode. Waitting for input a reset pose... " << endl;
      }
    }

    if(m_getCallBackPoseFlag)
    {
      m_getCallBackPoseFlag = false;
      m_pLidarLocation->setPose(m_callBackPose);
      m_status = normal;
    }

    return reflashState;


  }

  Status getStatus()
  {
    return m_status;
  }

  inline Eigen::Matrix4f getNewGlobalPose()
  {
    return *(m_vGlobalPose.end()-1);
  }

  inline Eigen::Matrix4f getNewGlobalDeltaPose()
  {
    return *(m_vGlobalDeltaPose.end()-1);
  }

  inline pcl::PCLHeader getNewHeader()
  {
    return *(m_vHeader.end()-1);
  }

   inline VPointCloud::Ptr getNewScan()
   {
     return m_pNewScan;
   }


   inline bool getEnableSwitchLidarData()
   {
     return m_enableSwitchLidarData;
   }

   inline bool getEnablePublishGlobalDenseMap()
   {
     return m_enablePublishGlobalDenseMap;
   }


   inline void getRecordData(std::vector<float> &data)
   {
     m_pState_machine_debug_record->getRecordData(data);
   }

   inline ros::Time getLidarHardTime()
   {
     return m_LidarHardTime;
   }

   inline Eigen::Matrix4f getImuPoseAtLidarPoseMoment()
   {
     return *(m_vImuPoseAtLidarPoseMoment.end()-1);
   }

   inline ros::Duration getHardSysAndRosSysdeltaTime()//deltaTime = systemTime-hardtime;
   {
     return m_pCorrectVlpXsensTool->deltaTime;
   }

   inline ros::Time getCurrentImuHardTime()
   {
     ros::Time temp;
     temp.fromNSec( *(m_pCorrectVlpXsensTool->transTime_us.end()-1) * 1000 );
     return temp;
   }

   inline ros::Time getCurrentImuRosTime()
   {
     ros::Time temp;
     temp.fromNSec( *(m_pCorrectVlpXsensTool->transTime_us.end()-1) * 1000 );
     return temp+m_pCorrectVlpXsensTool->deltaTime;
   }

   inline Eigen::Matrix4f getCurrentImuValue()
   {
     return *(m_pCorrectVlpXsensTool->transRT.end()-1);
   }


  locationV2::mapManager::Ptr m_pMapManager;
  locationV2::location::Ptr m_pLidarLocation;
  VelodyneCorrection::Ptr m_pCorrectionTool;
  correct_vlp_xsens::Ptr m_pCorrectVlpXsensTool;

  Status m_status;
  ImuStatus m_imuStatus;
  uint_fast64_t m_countLoop;
  unsigned int m_imuVaildCount;
  Eigen::Matrix4f m_callBackPose;//回调函数的pose
  Eigen::Matrix4f m_imuPoseAtLidarPoseMoment;//当前激光帧在触发时刻对应的imu值。
  boost::circular_buffer<Eigen::Matrix4f> m_vGlobalPose;//全局位置。
  boost::circular_buffer<Eigen::Matrix4f> m_vGlobalDeltaPose;
  boost::circular_buffer<pcl::PCLHeader> m_vHeader;
  boost::circular_buffer<Eigen::Matrix4f> m_vImuPoseAtLidarPoseMoment;

  VPointCloud::Ptr m_pNewScan;

  bool m_initCheck[2];//0 是pose 1是utc时间
  bool m_newScanFlag;
  bool m_getCallBackPoseFlag;
  bool m_enableSwitchLidarData;//是否使用拼接的稠密点云
  bool m_enablePublishGlobalDenseMap;//是否发布全局稠密地图，用于可视化，但是巨大的地图会导致rviz变卡顿。

  float m_recoverModeMaxCount;
  float m_recoverModeCounter;

  ros::Time m_LidarHardTime;

  state_machine_debug_record::Ptr m_pState_machine_debug_record;

};

}

#endif
