#ifndef _LOCATIONV2_HPP_
#define _LOCATIONV2_HPP_

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>
#include <pclomp/ndt_omp.h>
#include "3d_party_libraries.h"
#include <pcl/search/kdtree.h>
#include <pcl/common/transforms.h>
#include <location/tic_toc.h>
namespace locationV2
{
using namespace std;

class mapManager
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef boost::shared_ptr<mapManager> Ptr;

  mapManager(vector<VPointCloud> &mapList)
  {
    m_mapList.assign(mapList.begin(), m_mapList.end());
  }
  mapManager(VPointCloud map)
  {
    m_mapList.push_back(map);
  }

  VPointCloud::Ptr getMap(unsigned int id)
  {
    return boost::make_shared<VPointCloud>(m_mapList.at(id));
  }

  vector<VPointCloud> m_mapList;//地图序列，支持多子地图定位
};

class location
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef shared_ptr<location> Ptr;

  location(const VPointCloud::Ptr pCurrentMap,
           const float targetResolution=0.5,
           const float sourceResolution=1.0,
           const float fitnessMaxRange=1.0):
    m_pCurrentMap(pCurrentMap), m_targetResolution(targetResolution), m_sourceResolution(sourceResolution),
    m_fitnessMaxRange(fitnessMaxRange),
    m_newScanPose(Eigen::Isometry3f::Identity().matrix()), m_newScanDeltaPose(Eigen::Isometry3f::Identity().matrix())
  {

    m_currentNdtMap.setNumThreads(10);
    m_currentNdtMap.setNeighborhoodSearchMethod(pclomp::DIRECT7);
    m_currentNdtMap.setTransformationEpsilon (0.01);//设置迭代结束的条件
    m_currentNdtMap.setStepSize (0.1);//0.1改0.2没影响
    m_currentNdtMap.setResolution (m_targetResolution);//0.2在停车场只有10cm柱子的时候比较好，0.5会出现匹配问题
    m_currentNdtMap.setMaximumIterations (5);//30改成5 没影响,耗时不变，都是提前跳出的,但是对于初始位置给的不好的情况，会多次迭代，匹配时间大大增加
    m_currentNdtMap.setOulierRatio(0.15);
    m_currentNdtMap.setInputTarget (m_pCurrentMap);
    //cout << "waiting! init map" << endl;
    m_currentNdtMap.initCompute();//初始化map空间，耗时
    //cout << "init done" << endl;

    m_currentKDTree.setInputCloud(m_pCurrentMap);

  }

  inline float limitYaw(float value)
  {
    if(value>3.1415926)
      value -= 3.1415926;
    if(value < -3.1415926)
      value += 3.1415926;
    return value;
  }

  void showPose()
  {
    static int counter = 0;
    counter++;
    if(counter<20)
      return;
    counter = 0;
    //      Eigen::Matrix3f r_TransformAllmatrix = m_newScanPose.block<3,3>(0,0);
    //      Eigen::Vector3f r_ = r_TransformAllmatrix.eulerAngles(0,1,2);//roll pitch yaw 顺序

    const Eigen::Matrix3f rot(m_newScanPose.block<3,3>(0,0));
    const Eigen::Vector3f rot_x(rot * Eigen::Vector3f::UnitX());
    const float z_rotation = std::atan2(rot_x[1], rot_x[0]);

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "Global Pose (x, y, th_rad, th_degree): " << \
                 m_newScanPose(0, 3) << " " << m_newScanPose(1, 3) << " "<< z_rotation << " "<< z_rotation/3.14159*180 << std::endl;
    std::cout.unsetf( std::ios::fixed );
  }

  void updatePose(const VPointCloud::Ptr pNewScan)
  {
    m_pNewScan = pNewScan;//智能指针引用+1,不用拷贝对象。会自动释放之前的指针。
    VPointCloud::Ptr pFilteredCloud (new VPointCloud);
    downsampleCloud(m_pNewScan, pFilteredCloud, m_sourceResolution);
    m_pNewDownSampleScan = pFilteredCloud;
    m_currentNdtMap.setInputSource (m_pNewDownSampleScan);

    // Calculating required rigid transform to align the input cloud to the target cloud.
    VPointCloud::Ptr pOutputCloud (new VPointCloud);
    m_currentNdtMap.align (*pOutputCloud, m_newScanPose * m_newScanDeltaPose);//加上上一帧的预测优化迭代速度
    //m_currentNdtMap.align (*pOutputCloud, m_newScanPose);//加上上一帧的预测优化迭代速度

    m_pNewDownSampleScanAfterTransform = pOutputCloud;
    m_newScanDeltaPose = m_newScanPose.inverse() * m_currentNdtMap.getFinalTransformation();
    m_newScanPose = m_currentNdtMap.getFinalTransformation();

    debug_record[0] = m_currentNdtMap.getFinalNumIteration();
    debug_record[1] = m_currentNdtMap.getTransformationProbability();

    showPose();
  }

  void updatePose(const VPointCloud::Ptr pNewScan, const Eigen::Matrix4f &deltaPose)
  {
    m_pNewScan = pNewScan;//智能指针引用+1,不用拷贝对象。会自动释放之前的指针。
    VPointCloud::Ptr pFilteredCloud (new VPointCloud);
    downsampleCloud(m_pNewScan, pFilteredCloud, m_sourceResolution);
    m_pNewDownSampleScan = pFilteredCloud;
    m_currentNdtMap.setInputSource (m_pNewDownSampleScan);


    // Calculating required rigid transform to align the input cloud to the target cloud.
    VPointCloud::Ptr pOutputCloud (new VPointCloud);
    m_currentNdtMap.align (*pOutputCloud, m_newScanPose * deltaPose);//加上上一帧的预测优化迭代速度

    m_pNewDownSampleScanAfterTransform = pOutputCloud;
    m_newScanDeltaPose = m_newScanPose.inverse() * m_currentNdtMap.getFinalTransformation();
    m_newScanPose = m_currentNdtMap.getFinalTransformation();

    debug_record[0] = m_currentNdtMap.getFinalNumIteration();
    debug_record[1] = m_currentNdtMap.getTransformationProbability();

    showPose();
  }

  void setPose(Eigen::Matrix4f newScanPose)
  {
      m_newScanPose = newScanPose;
      m_newScanDeltaPose = Eigen::Isometry3f::Identity().matrix();
  }



  /**
   * @brief calc_fitness_score
   * 输入点云 和地图点云匹配，衡量误差
   * @param pInputScan
   * @param average_lenth
   * @param valid_scale
   * @return
   */
  bool calc_fitness_score(const VPointCloud::Ptr pInputScan, float &average_lenth, float &valid_scale) const {

    TicToc timer;
    float fitness_score = 0.0;

    std::vector<int> nn_indices (1);
    std::vector<float> nn_dists (1);

    // For each point in the source dataset
    int nr = 0;
    //可以考虑使用omp，或者降低采样点数
    for (size_t i = 0; i < pInputScan->points.size (); ++i)
    {
      // Find its nearest neighbor in the target
      m_currentKDTree.nearestKSearch (pInputScan->points[i], 1, nn_indices, nn_dists);

      // Deal with occlusions (incomplete targets)
      if (nn_dists[0] <= m_fitnessMaxRange)
      {
        // Add to the fitness score
        fitness_score += nn_dists[0];
        nr++;
      }
    }


//    cout << "spend time " << timer.toc() << "ms."
//         << " average lenth " << fitness_score/nr << " scale " <<  (float)nr/(float)m_pNewDownSampleScanAfterTransform->points.size () << " , "
//         << nr << "/" << m_pNewDownSampleScanAfterTransform->points.size () <<endl;

    //在玉泉50cm降采样地图中，m_fitnessMaxRange=1.0,正确定位下：average_lenth在0.12以下0.06居多，fitness_score在0.96以上
    //在玉泉50cm降采样地图中，m_fitnessMaxRange=1.0,错误定位下：average_lenth在0.20以上，fitness_score在0.85以下
    if (nr > 0)
    {
      average_lenth = fitness_score/nr;
      valid_scale = (float)nr/(float)pInputScan->points.size ();
      return true;
    }
    else
    {
      return false;
    }
  }


  bool calc_fitness_score(float &average_lenth, float &valid_scale) const {

    return calc_fitness_score(m_pNewDownSampleScanAfterTransform, average_lenth, valid_scale);

  }

  /**
   * @brief recoverPose
   * 定位出错后用随机撒点的方式恢复位置。这个函数会很耗时。
   * 考虑优先使用猜测的位姿，在查找的过程中，监测评价函数，当分值较高时就提前退出。
   * @param pNewScan 当前帧
   * @param guessPose 预测的位置
   * @param sampleRadius 不确定的半径范围
   * @param sampleDegree 不确定的角度范围（现在支持水平）
   * @param resultPose 输出的位置 列表
   * @param average_lenth 输出的评价距离 列表
   * @param valid_scale 输出的评价有效点比例 列表
   */
  void recoverPose(const VPointCloud::Ptr pNewScan, const Eigen::Matrix4f &guessPose, const float &sampleRadius, const float &sampleDegree,
                   vector<Eigen::Matrix4f> &resultPose, vector<float> &averageLenth, vector<float> &validScale)
  {
    TicToc timer;
    resultPose.clear();
    averageLenth.clear();
    validScale.clear();

    static float stepLenth = 2.0;
    static float stepDegree = 15;

    float maxLenth = fabs(sampleRadius)+0.5;
    float maxDegree = fabs(sampleDegree)+0.5;

    int stepNumberLenth = maxLenth/stepLenth;
    int stepNumberDegree = maxDegree/stepDegree;

    VPointCloud::Ptr pFilteredCloud (new VPointCloud);
    downsampleCloud(pNewScan, pFilteredCloud, m_sourceResolution);
    m_currentNdtMap.setInputSource (pFilteredCloud);

    for(int x=-stepNumberLenth; x<=stepNumberLenth; x++)
    {
      for(int y=-stepNumberLenth; y<=stepNumberLenth; y++ )
      {
        for(int degree=-stepNumberDegree; degree<=stepNumberDegree; degree++)
        {
          Eigen::Matrix4f temp(Eigen::Isometry3f::Identity().matrix());
          temp(0,3) = x*stepLenth;
          temp(1,3) = y*stepLenth;
          Eigen::AngleAxisf t_V(degree*stepDegree/180*3.14159, Eigen::Vector3f(0, 0, 1));
          temp.block<3,3>(0,0) = t_V.matrix();

          VPointCloud::Ptr pOutputCloud (new VPointCloud);
          m_currentNdtMap.align (*pOutputCloud, guessPose * temp);

          float _averageLenth, _validScale;
          if(calc_fitness_score(pOutputCloud, _averageLenth, _validScale))
          {
            averageLenth.push_back(_averageLenth);
            validScale.push_back(_validScale);
          }
          resultPose.push_back(m_currentNdtMap.getFinalTransformation());
          if(validScale.back()>0.95)//提前跳出
          {
            cout << "spend time " << timer.toc() << "ms. stepNumberLenth stepNumberDegree: " << stepNumberLenth << " " << stepNumberDegree <<endl;
            return;
          }
        }
      }
    }
    cout << "spend time " << timer.toc() << "ms. stepNumberLenth stepNumberDegree: " << stepNumberLenth << " " << stepNumberDegree <<endl;
  }

  inline Eigen::Matrix4f getnewScanPose()
  {
    return m_newScanPose;
  }

  inline Eigen::Matrix4f getnewScanDeltaPose()
  {
    return m_newScanDeltaPose;
  }

  inline float* getDebugParam()
  {
    return debug_record;
  }

  VPointCloud::Ptr m_pCurrentMap;//全局地图
  const float m_targetResolution;//被匹配点云的ndt栅格分辨率，米，室内0.5,室外1比较合适。不要使用大于1的，定位会不精确。
  const float m_sourceResolution;//输入点云降采样，米，室内1,室外2比较合适。
  const float m_fitnessMaxRange;//定位有效性检测中的最大距离限制。

  //ndt匹配后的结果
  //0 迭代次数、1  匹配正确的概率
  float debug_record[5];


  Eigen::Matrix4f m_newScanPose;
  Eigen::Matrix4f m_newScanDeltaPose;

  VPointCloud::Ptr m_pNewScan;
  VPointCloud::Ptr m_pNewDownSampleScan;
  VPointCloud::Ptr m_pNewDownSampleScanAfterTransform;//匹配后的点云（已降采样）

  //pcl::NormalDistributionsTransform<VPoint, VPoint> ndt;
  pclomp::NormalDistributionsTransform<VPoint, VPoint> m_currentNdtMap;

  pcl::search::KdTree<VPoint> m_currentKDTree;



  //just for test
  void test_theoretically_fitness_score3D(float cellSize)
  {
    float step = 0.001;//1mm采样
    float half_cellSize = cellSize/2;
    float lenth=0;
    unsigned int num=0;
    for(float x=0; x<=half_cellSize; x+=step)
      for(float y=0; y<=half_cellSize; y+=step)
        for(float z=0; z<=half_cellSize; z+=step)
        {
          lenth += sqrt(x*x+y*y+z*z);
          num++;
        }
    cout << "ave_lenth 3D: " << lenth/num <<endl;
    //当cellSize=0.5时，lenth/num=0.23
  }

  void test_theoretically_fitness_score2D(float cellSize)
  {
    float step = 0.001;//1mm采样
    float half_cellSize = cellSize/2;
    float lenth=0;
    unsigned int num=0;
    for(float x=0; x<=half_cellSize; x+=step)
      for(float y=0; y<=half_cellSize; y+=step)
      {
        lenth += sqrt(x*x+y*y);
        num++;
      }
    cout << "ave_lenth 2D: " << lenth/num <<endl;
    //当当cellSize=0.5时，lenth/num=0.19
  }

  /**
   * @brief test_memory
   * 测试智能指针的释放。
   */
  void test_memory()
  {
    VPointCloud::Ptr p_data1(new VPointCloud());
    VPointCloud::Ptr p_data2(new VPointCloud());

    cout << "step1 " <<"p_data1: " << p_data1.use_count() << " p_data2: " << p_data2.use_count() <<endl;

    {
      VPointCloud::Ptr p_temp = p_data1;
      cout << "step2 " <<"p_data1: " << p_data1.use_count() << " p_data2: " << p_data2.use_count() <<endl;


      p_temp = p_data2;//会自动释放之前的指针，再获取当前的指针。
      cout << "step3 " <<"p_data1: " << p_data1.use_count() << " p_data2: " << p_data2.use_count() <<endl;
    }

    cout << "step4 " <<  "p_data1: " << p_data1.use_count() << " p_data2: " << p_data2.use_count() <<endl;

    //  step1 p_data1: 1 p_data2: 1
    //  step2 p_data1: 2 p_data2: 1
    //  step3 p_data1: 1 p_data2: 2
    //  step4 p_data1: 1 p_data2: 1
  }



};


}





#endif
