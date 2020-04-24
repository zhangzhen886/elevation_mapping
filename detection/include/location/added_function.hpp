#ifndef _ADDED_FUNCTION_H_
#define _ADDED_FUNCTION_H_

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>
#include <pcl/search/kdtree.h>
#include <boost/circular_buffer.hpp>
#include <boost/shared_ptr.hpp>
#include <location/tic_toc.h>
#include <location/3d_party_libraries.h>

namespace addedFunction {


class switchPointsTool
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef boost::shared_ptr<switchPointsTool> Ptr;

  switchPointsTool(unsigned int bufferSize):
    m_switchPointClondPtr(bufferSize),
    m_pSwitchedPoints(new VPointCloud())
  {
  }

  /**
   * @brief update 更新滑动窗口
   * @param pNewScan 当前帧
   * @param newPose 当前帧的全局位置
   */
  void update(const VPointCloud::Ptr pNewScan, const Eigen::Matrix4f &newPose)
  {

    VPointCloud::Ptr temp(new VPointCloud());
    pcl::transformPointCloud (*pNewScan, *temp, newPose);

    m_switchPointClondPtr.push_back(temp);

    //有更好的拼接方案码？
    m_pSwitchedPoints->clear();
    for(auto it_PC = m_switchPointClondPtr.begin(); it_PC!=m_switchPointClondPtr.end(); it_PC++)
    {
      *m_pSwitchedPoints += *(*it_PC);
    }

    pcl::transformPointCloud (*m_pSwitchedPoints, *m_pSwitchedPoints, newPose.inverse());
  }

  VPointCloud::Ptr getSwitchedPoints()
  {
    return m_pSwitchedPoints;
  }

protected:
  boost::circular_buffer<VPointCloud::Ptr> m_switchPointClondPtr;
  VPointCloud::Ptr m_pSwitchedPoints;


};

}


#endif
