/*
 * Copyright (c) 2017, <copyright holder> <email>
 * All rights reserved.
 * 
 * 最完美的应该是，考虑各种特征，包括角点和直线
 * 先检测角点, 然后再检测直线
 * 由于点密度的影响,所以插值也是必须的
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

#ifndef EXTRACTGEOMETRYFEATURE_H
#define EXTRACTGEOMETRYFEATURE_H
#include "../io/scan_reader.h"
#include <Eigen/Dense>
#include <boost/graph/graph_concepts.hpp>
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/kdtree/kdtree.h>
//#include <pcl/segmentation/extract_clusters.h>
#include "../common/math_supplement.h"
#include <falkolib/Feature/FALKO.h>
#include<falkolib/Feature/FALKOExtractor.h>
#include<flirtlib/feature/RangeDetector.h>
//#include<flirtlib/feature/NormalEdgeDetector.h>
//#include<flirtlib/feature/CurvatureDetector.h>
#include<flirtlib/feature/InterestPoint.h>
#include<flirtlib/sensors/LaserReading.h>
#include<flirtlib/utils/PeakFinder.h>
#include <pcl/kdtree/kdtree_flann.h>


  #define DAL_DEBUG false

  #define DALKO_DEBUG(msg) if(DAL_DEBUG) std::cout<<msg



namespace geomeasurer
{

  class keypoint
  {
  public:
    keypoint();
    keypoint(const int& index_, const double &score_);
    static bool compare(const keypoint& kp1, const keypoint& kp2);
    int index;
    double score;
  };
 
  typedef pcl::PointIndices ClusterIndices;
  typedef std::vector<keypoint> KeyPoints;
  typedef PointCloud featurePointSet;
  typedef Eigen::MatrixXf Discriptors;
  typedef std::vector<int> NeighIndices;
   
  class extractGeometryFeature
{
public:
  extractGeometryFeature();
  extractGeometryFeature(const sensor::rangeData & ranges_data);
  extractGeometryFeature(const PointCloud & pcd);
  void setSlideWindowSizes(const int &slide_window_sizes_);
  void setRegionGrowRadius(const double & region_grow_radius_);
  void setClusterMinSize(const uint32_t & cluster_min_size_);
  void setMinIncludedAngle(const double min_angle_);
  void setNeigh_a(const double & a_);
  void setNeigh_b(const double & b_);
  void setIsClustering(const bool & IsClustering_);
  void setNMSRadius(const double & NMS_radius_);
  void setBeta(const double &beta);
  void setGridSetorNum(const int gridSectorsNum_);
  void setMinNeighNum(const int &min_point_num_);
  void setMinAloneNum(const int &alone_);
  void setDesSectorNum(const int & dscpSectorsNum_);  
  void setMatchDistThres(const double & match_dist_thres_);
  void setMatchDistinctRatio(const double distinct_ratio_);
  void setMatchMinScore(const double match_score_min_);
  void ChooseMethodofComputingOrientation(const bool & keypoints_based_method_);
//     int dscpSectorsNum=144;//144, 288
//   
//   double match_dist_thres=0.18;
//    
//   double distinct_ratio=1.0;
//   
//   double match_score_min=40;
//   
//   bool keypoints_based_method=false;
//   
  void setInputRanges(const sensor::rangeData & ranges_);
  featurePointSet extractgfs(std::string T);
  featurePointSet extractFAKLO();
  featurePointSet extractCornerWithAreaTsensor(const ClusterIndices &cluster);
  featurePointSet extractCornerWithimprovedFAKLO(const ClusterIndices &cluster);
  /**
   * @brief 返回一次激光数据中的特征点，note，必须先执行提取步骤，否则为空
   */
  KeyPoints getKeypoints();
  
  /**
   * @brief 获取激光描述子
   */
  Discriptors getGCdiscriptor();
  
  /**
   * @brief 已知索引，返回3D坐标
   */
  point3d GetPoint3dfromIndex(const int &pointIndex);
  
  std::vector<std::tuple<int,int,double> > match(const KeyPoints &kps, const Discriptors &GCS);
 //TODO
 featurePointSet extractFLIRT();

private:


  void clear();
  
  /**
   * @brief 孤立特征点抑制
   * @return 抑制后特征点构成的点云
   */
  featurePointSet alonePointSupress();
  /**
   * @brief 非极大值抑制
   * @return 抑制后特征点构成的点云
   */
   featurePointSet NonMaxSupress();
   /**
    * @brief 使用欧式距离进行聚类
    */
  std::vector<pcl::PointIndices> EuclideanClusterExtraction() const;
  
    /**
   * @brief 把特征点到点云API
   */
  featurePointSet fromKeypoints();
  
  featurePointSet fromAllkeypoints();
  
  
  /**
   * @brief 计算每一个点应该有的邻阈的大小
   *@param range 激光点的距离
   *@return 邻阈的半径 
   */
  float getNeiborhood(const double &range);
  
  
  /**
   * @brief 根据中点索引，以及邻域半径计算邻域内的点成员
   * @param numberofCenter2clusterboundary 中点到类的边界点的个数
   * @param center_index 中心点索引
   * @param radius 邻域半径
   * @param[out] neigh 返回计算到的邻域点集合
   * @param[in] flag_Left 如果是计算左边邻域则设为true,计算右边设置为false
   */
void getNeiborhoodMember(const int &numberofCenter2clusterboundary, const size_t& center_index, const double& radius, std::vector< point3d >& neigh, bool flag_Left);
  
  /**
   * @brief 对邻阈内的点的散度进行打分
   * @param center 中心点
   * @param singleSideNeigh 某一边邻域内的点
   * @return 分散打分，越分散分越大
   */
  double EvaulateNeighDivergence(const point3d & center, const std::vector<point3d>& singleSideNeigh);
  
  
  /**
   * @brief 计算点在栅格上的位置
   * @param center 中心点 
   * @param singleSideNeigh 邻域内点
   * @return 栅格位置，与之前点的顺序对应
   */
  std::vector<int> computeNeighPlorDistribution(const point3d & center, const std::vector<point3d>& singleSideNeigh);
  
  
  /**
   * @brief 联合评估当前特征点的打分
   * @param[in] center 当前特征点三位坐标
   * @param[in] LeftNeigh 左邻域内的点
   * @param[in] rightNeigh 右邻域内的点
   * @return 影响角度的均值、协方差度、KL散度
   */
 std::tuple< double, double, double> jointEvaluate(const point3d &center, const std::vector<point3d>&LeftNeigh, const std::vector<point3d>& rightNeigh);
  
  double EvaulateAngleQuality(const point3d &center, const std::vector<point3d>&LeftNeigh, const std::vector<point3d>& rightNeigh);
  
  /**
   * @brief 评估特征点在尺度上的不变性，尺度通过距离响应端点距离中心点远近来判断
   */
std::pair<double,double> EvaulateInvarianceNeigh(const point3d &center, const std::vector<point3d>&LeftNeigh, const std::vector<point3d>& rightNeigh);
  

/**
   * @brief 越接近直角的特征点越被看好, 为这类特征点一些额外的奖励。
   */
  double extra_reward_from(const double &area, const double& area_min);
  
  /**
   * @brief 已知三个端点，计算三角形面积
   */
  double computeAreaGivenEndpointCoordinates(const point3d& A1,const point3d &A2, const point3d& A3);
  
  
  /**
   * @brief 基于邻域点计算主方向（实际采用计算角平分线）
   */
  int getCornerOrientationfromNeigh(const int& index);
  
  
  /**
   * @brief 基于特征点计算主方向
   */
  int getCornerOrientationBasedonKeypoints(const int& index);
  
  
  int distanceofSectors(const int &index1, const int & index2);  
  
  /**
   * @brief 计算面积的阈值
   * @return 返回面积的阈值
   */
  double getAreaThres();
  
    /**
   * @brief 计算面积的阈值
   * @return 返回面积的阈值
   */
  double getAreaThres(double edge1, double edge2);
  
  
  /**
   * @brief 效果不好，删除不用
   */
  featurePointSet  extractgfsWithoutCluster()=delete;
  
  /**
   * @brief 使用LOAM中的思想
   * @test 效果不好，删除不用 
   */
  featurePointSet extractFeatureFromCluster(const ClusterIndices & cluster)=delete;
  
  /**
   * @brief 选择一个聚类好的线条的两个端点，然后计算中间点在与这两个端点张成的面积
   * @test 效果不好，删除不用
   */
  featurePointSet extractfromTensorField(const ClusterIndices &cluster);
  
  
  /**
   * @brief 自适应的调整响应因子
   * @param[in] ri 当前点的距离
   * @return 返回响应因子
   */
  
  double adaptiveResponsefactor(const double & ri);



  sensor::rangeData ranges;
  PointCloud candiate_pcd;
  KeyPoints keypoints;
  KeyPoints Allkeypoints;  
  PointCloud keypointPcd;
  pcl::KdTreeFLANN<pcl::PointXYZ>  kdtree_keypoints;   

  
  
  bool isCreatedKdtree=false;
  
  bool IsClustering=true;
 
double ratio_invariance=0.6;

 double min_angle=30;
 
 //这个参数值实际运行时会被覆盖
 double area_min_size=0.005;
 
 //左右邻域点最少的个数
 int min_point_num=1;
  //最小孤立特征点个数
 int alone=1; 
 double alone_radius=0.15;  
  //NMS
 double NMS_radius=0.2;    
  //区域增长, 聚类
 uint32_t cluster_min_size=4; 
 double region_grow_radius=0.2;
  //一下三个参数用来计算邻阈半径
  float ri;
  float a=0.2;
  float b=0.07;
  
  double minValPercent=0;
  
 int gridSectorsNum=36; 
 
 //直角奖励系数 
 double gainClosingRightAngle=10;
  
  //没用
  float b_ratio=2.5;
  
   //没有使用
 double feature_response_threshold=0.1;
 
 //没有使用
  int slide_window_sizes=6;  
  
  //

  
 
  Eigen::MatrixXf GCdiscriptors;
  
  int dscpSectorsNum=144;//144, 288
  
  double match_dist_thres=0.18;
   
  double distinct_ratio=1.0;
  
  double match_score_min=40;
  
  bool keypoints_based_method=false;
};

}



#endif // EXTRACTGEOMETRYFEATURE_H
