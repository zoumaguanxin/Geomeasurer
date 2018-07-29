/*
 * Copyright (c) 2018, <copyright holder> <email>
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY <copyright holder> <email> ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <copyright holder> <email> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */

#ifndef REPEATABILITYTEST_H
#define REPEATABILITYTEST_H

#include "../transform/pose2d.h"
#include "../sensor/scan_data.h"

#include "../bridge/sensor_bridge.h"

#include "../transform/transform.h"

#include "../viz/viz.h"

#include "../extract_geometry_feature/extractgeometryfeature.h"

#include <flirtlib/feature//CurvatureDetector.h>
#include <flirtlib/feature/Detector.h>

#include <pcl/kdtree/flann.h>

#include <falkolib/Feature/FALKO.h>

#include <utils/SimpleMinMaxPeakFinder.h>

#define re_debug false

#define ReTest_DEBUG(msg) if(re_debug) std::cout<<msg

namespace geomeasurer{ 
  
  
  //在基类中对纯虚函数定义的函数体的调用，必须通过“基类名::函数名（参数表）”的形式
class RepeatabilityTest
{
public:
RepeatabilityTest();

RepeatabilityTest(const int& k_, const double& max_hausdorff_distance_);

void setTwoRangesWithPose(const sensor::rangeWithpose& rwp_ref, const  sensor::rangeWithpose& rwp_src);

void setKpAssociationDistThres(const double &kp_association_dist_thres_);

void setMatchTrue();

void setDetectedTrue();

RepeatabilityTest(const RepeatabilityTest& other);

virtual double getRepeatabilityDetecor();

double getRepeatabilityDescriptos();

virtual int getLatentCorrespondingsNum(const std::vector<int> &unassociatedSrcIndexes, const std::vector<int> &unassociatedRefIndexes);

int getCorrespondingsNum();

pose2d computeRelativePose();
 virtual bool detect()=0;
 virtual int match()=0;
~RepeatabilityTest();
RepeatabilityTest& operator=(const RepeatabilityTest& other);
bool operator==(const RepeatabilityTest& other);

 std::vector<point3d> getSuportRegionsfrompcd(const int &index,PointCloud::ConstPtr pcd_ptr);

 /**
   * @brief 根据中点索引，以及邻域半径计算邻域内的点成员
   * @param numberofCenter2clusterboundary 中点到类的边界点的个数
   * @param center_index 中心点索引
   * @param radius 邻域半径
   * @param[out] neigh 返回计算到的邻域点集合
   * @param[in] flag_Left 如果是计算左边邻域则设为true,计算右边设置为false
   */
void getNeiborhoodMember(PointCloud::ConstPtr pcd_ptr, const size_t& center_index, const double& radius, std::vector< point3d >& neigh, bool flag_Left);

  sensor::rangeData Range_ref;
  sensor::rangeData Range_src;
  
   PointCloud ref_kps;
  PointCloud src_kps;
  PointCloud rgsteredsrc_kps;
  KeyPoints kp_kps_ref;
  KeyPoints kp_kps_src;
  PointCloud src_pcd;
  PointCloud ref_pcd;
  PointCloud rgsteredsrc_pcd;
  pose2d relativePose;
  
  std::vector<std::pair<int,int>> groundKpPairs;
  sensor::rangeWithpose Ref_rangewithpose;
  sensor::rangeWithpose Src_rangewithpose;
  std::vector<std::pair<int,int>> matchedKpPairs;
  

  int K;
  double a=0.2;
  double b=0.07;
  double max_hausdorff_distance;  
  
  bool ismathched=false;
  
  bool isdetected=false;
  
  double kp_association_dist_thres;
};


class RepeatabilityTestDALKO:public RepeatabilityTest
{
  
  public:
  RepeatabilityTestDALKO(){}
  RepeatabilityTestDALKO(const int& k_, const double& max_hausdorff_distance_ ):RepeatabilityTest(k_,max_hausdorff_distance_){}
  virtual bool detect();
  virtual int match();  
  private:
    extractGeometryFeature gfs;
    geomeasurer::Discriptors ref_kps_des;
    geomeasurer::Discriptors src_kps_des;
};


class RepeatabilityTestFALKO:public RepeatabilityTest
{
public:
RepeatabilityTestFALKO(){}
RepeatabilityTestFALKO(const int& k_, const double& max_hausdorff_distance_):RepeatabilityTest(k_,max_hausdorff_distance_){}
    virtual bool detect();
    virtual int match();
private:
 
  
};


class RepeatabilityTestFLIRT:public RepeatabilityTest
{
public:
  
RepeatabilityTestFLIRT(){};
RepeatabilityTestFLIRT(const int& k_, const double& max_hausdorff_distance_):RepeatabilityTest(k_,max_hausdorff_distance_){}
 virtual bool detect();
 virtual int match();
 virtual double getRepeatabilityDetecor();
 virtual int getLatentCorrespondingsNum(const std::vector<int> &unassociatedSrcIndexes, const std::vector<int> &unassociatedRefIndexes);
  std::vector<InterestPoint*> keypoints_src,keypoints_ref;
};

}

#endif // REPEATABILITYTEST_H
