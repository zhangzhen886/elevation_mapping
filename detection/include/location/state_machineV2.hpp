#ifndef _STATE_MACHINEV2_H_
#define _STATE_MACHINEV2_H_

#include <boost/circular_buffer.hpp>
#include <boost/shared_ptr.hpp>
#include <location/locationV2.hpp>
#include <location/state_machine_debug_record.hpp>

namespace state_machineV2 {


class state_machine
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef boost::shared_ptr<state_machine> Ptr;

  enum Status
  {
    normal=0,
    recover=1,
    error=2
  };

  state_machine(VPointCloud::Ptr pMap ,
                float targetResolution=0.5,
                float sourceResolution=1.0,
                float fitnessMaxRange=1.0,
                bool enableSwitchLidarData=false,
                bool enablePublishGlobalDenseMap=false):
    m_pMapManager(new locationV2::mapManager(*pMap)),
    m_pLidarLocation(new locationV2::location(m_pMapManager->getMap(0),
                                              targetResolution,
                                              sourceResolution,
                                              fitnessMaxRange)),
    m_status(Status::normal), m_countLoop(0),
    m_newScanFlag(false),
    m_getCallBackPoseFlag(false),
    m_enableSwitchLidarData(enableSwitchLidarData),
    m_enablePublishGlobalDenseMap(enablePublishGlobalDenseMap),
    m_vGlobalPose(100), m_vGlobalDeltaPose(100), m_vHeader(100),
    m_recoverModeMaxCount(10), m_recoverModeCounter(0),
    m_pState_machine_debug_record(new state_machine_debug_record())

  {

  }



  void updataCallBackPose(Eigen::Matrix4f &pose)
  {
    m_callBackPose = pose;
    m_getCallBackPoseFlag = true;
  }

  void updataLidarData(VPointCloud::Ptr pNewScan)
  {
    VPointCloud::Ptr temp(new VPointCloud());
    removePointsUpTo(pNewScan, temp, 1, 50);

    m_pNewScan = temp;
    m_newScanFlag = true;
  }
  /**
   * @brief initDone
   * 等待初始化完成
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

      return true;
    }
    return false;
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
        //送入当前帧，更新位置
        m_pState_machine_debug_record->timer_start();
        m_pLidarLocation->updatePose(m_pNewScan);
        m_pState_machine_debug_record->time_stop(state_machine_debug_record::recordType::ndtMatchingTimeWithoutIMU);

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

          if(valid_scale<0.20)//判断条件
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


        m_vGlobalPose.push_back(m_pLidarLocation->getnewScanPose());
        m_vGlobalDeltaPose.push_back(m_pLidarLocation->getnewScanDeltaPose());
        m_vHeader.push_back(m_pNewScan->header);
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

   /**
    * @brief isLidarDataTimeOut
    * 检测激光数据是否异常
    * @return
    */
   inline bool isLidarDataTimeOut()
   {
     if(ros::Time::now() - pcl_conversions::fromPCL( (*(m_vHeader.end()-1)).stamp ) >
        ros::Duration(0.5))
     {
       cout << "Lidar timeOut!" << endl;
       return true;
     }
     return false;
   }


  locationV2::mapManager::Ptr m_pMapManager;
  locationV2::location::Ptr m_pLidarLocation;
  Status m_status;

  uint_fast64_t m_countLoop;
  Eigen::Matrix4f m_callBackPose;//回调函数的pose
  boost::circular_buffer<Eigen::Matrix4f> m_vGlobalPose;//全局位置。
  boost::circular_buffer<Eigen::Matrix4f> m_vGlobalDeltaPose;
  boost::circular_buffer<pcl::PCLHeader> m_vHeader;

  VPointCloud::Ptr m_pNewScan;

  bool m_newScanFlag;
  bool m_getCallBackPoseFlag;
  bool m_enableSwitchLidarData;//是否使用拼接的稠密点云
  bool m_enablePublishGlobalDenseMap;//是否发布全局稠密地图，用于可视化，但是巨大的地图会导致rviz变卡顿。

  float m_recoverModeMaxCount;
  float m_recoverModeCounter;

  state_machine_debug_record::Ptr m_pState_machine_debug_record;

};
}

#endif
