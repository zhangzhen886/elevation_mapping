#ifndef _STATE_MACHINE_DEBUG_RECORD_H_
#define _STATE_MACHINE_DEBUG_RECORD_H_

#include <iostream>
#include <string>
#include <map>
#include <boost/shared_ptr.hpp>
#include <Eigen/StdVector>
#include <location/tic_toc.h>

namespace state_machineV2 {


class state_machine_debug_record
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef boost::shared_ptr<state_machine_debug_record> Ptr;

  enum recordType
  {
    upackTime,//0 点云解压时间
    ndtMatchingTimeWithoutIMU,//1 点云匹配时间 无imu预测，帧间预测
    ndtMatchingTimeWithIMU,//2 点云匹配时间 有imu预测
    ndtCalFitnessTime,//3 点云评价时间
    ndtFinalNumIteration,//4 ndt迭代次数
    ndtTransformationProbability,//5 ndt匹配概率 应该不准
    ndtFitnessAverageLenth,//6 ndt检测 降采样后的点云和地图的匹配距离
    ndtFitnessValidScale,//7 ndt检测 降采样后的点云和地图的有效匹配比例
    MAX_COUNT//无意义，用于迭代
    //还要增加 delta角度 当前点云个数
  };

  state_machine_debug_record()
  {
    for(int i=upackTime; i<MAX_COUNT; ++i)
    {
      m_recordData.insert(std::pair<int, float>(i, 0));
    }
  }

  inline void timer_start()
  {
    m_timer.tic();
  }

  inline void time_stop(const recordType name)
  {
    m_recordData[name] = m_timer.toc();
  }

  inline void updataParam(const recordType name, float data)
  {
    m_recordData[name] = data;
  }

  inline void getRecordData(std::vector<float> &data)
  {
    data.clear();
    for(int i=upackTime; i<MAX_COUNT; i++)
    {
      data.push_back(m_recordData[i]);
    }
  }



  std::map<int, float> m_recordData;
  TicToc m_timer;

};
}
#endif
