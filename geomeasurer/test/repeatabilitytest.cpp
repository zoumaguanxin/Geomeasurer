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

#include "repeatabilitytest.h"
//#include "../viz/viz.hpp"
#include <utility>

namespace geomeasurer {
  RepeatabilityTest::RepeatabilityTest()
{

}

RepeatabilityTest::RepeatabilityTest(const int& k_, const double& max_hausdorff_distance_)
{
   K=k_;
   max_hausdorff_distance=max_hausdorff_distance_;
}


RepeatabilityTest::RepeatabilityTest(const RepeatabilityTest& other)
{
  
}

RepeatabilityTest::~RepeatabilityTest()
{

}

RepeatabilityTest& RepeatabilityTest::operator=(const RepeatabilityTest& other)
{

}

bool RepeatabilityTest::operator==(const RepeatabilityTest& other)
{

}

std::vector< point3d> RepeatabilityTest::getSuportRegionsfrompcd(const int& index, PointCloud::ConstPtr  pcd_ptr )
{
  point3d ceneter=pcd_ptr->points[index];
  double range=math::rangefromcartesian(ceneter);
  
   double ri=a*exp(b*range);
  
   ReTest_DEBUG("neigh radius:"<<ri<<std::endl);
   std::vector<point3d> neigh;
  getNeiborhoodMember(pcd_ptr,index,ri,neigh,true);
  getNeiborhoodMember(pcd_ptr,index,ri,neigh,false);
   
//   std::vector<point3d> tem;
//    int bi=K/float(2);
//    int start_pos;
//    if((index-bi)<0)
//    {
//      start_pos=0;
//   }
//   else
//   {
//     start_pos=index-bi;
//   }
//    for(int i=start_pos;i<start_pos+K;i++)
//    {
//     tem.push_back(pcd_ptr->points[i]); 
//   }
  assert(!neigh.empty());
  return neigh;
}



void RepeatabilityTest::getNeiborhoodMember(PointCloud::ConstPtr pcd_ptr, const size_t& center_index, const double& radius, std::vector< point3d >& neigh, bool flag_Left)
{
  int numberofCenter2clusterboundary;  
  if(flag_Left) numberofCenter2clusterboundary=center_index;
  else numberofCenter2clusterboundary=pcd_ptr->size()-center_index-1;
  double growing_dl;
    int j=0;
     while(1)
     {
       if(flag_Left)
       {
       j++;
       }
       else 
       {
	 j--;
      }
        if(std::abs(j)<=numberofCenter2clusterboundary)
       {	
	 growing_dl=math::pointDistance(pcd_ptr->points[center_index],pcd_ptr->points[center_index-j]);	 
	 if(growing_dl<radius)
	 {
	    neigh.push_back(pcd_ptr->points[center_index-j]);
	 }
	 else{
	   break;
	}
       }
       else
       {
	 break;
      }
    }  
}



void RepeatabilityTest::setTwoRangesWithPose(const sensor::rangeWithpose& rwp_ref, const sensor::rangeWithpose& rwp_src)
{
  
      
    assert(!rwp_ref.second.ranges.empty());
    Ref_rangewithpose.first=rwp_ref.first;
    Src_rangewithpose.first=rwp_src.first;
    
    Ref_rangewithpose.second=rwp_ref.second;
    Src_rangewithpose.second=rwp_src.second;
    Range_ref=rwp_ref.second;
    assert(!Range_ref.ranges.empty());
    Range_src=rwp_src.second;
    ref_pcd.clear();src_pcd.clear();
    ref_pcd=sensor::fromRangeData(Range_ref);
    src_pcd=sensor::fromRangeData(Range_src);
}


void RepeatabilityTest::setKpAssociationDistThres(const double& kp_association_dist_thres_)
{
   kp_association_dist_thres=kp_association_dist_thres_;
}


pose2d RepeatabilityTest::computeRelativePose()
{
  
   pose2dStamped pose2dsp_ref=  Ref_rangewithpose.first;
   pose2dStamped pose2dsp_src= Src_rangewithpose.first;
   Eigen::Matrix3f R_ref=pose2dsp_ref.getRoationMatrix(); 
   Eigen::Matrix3f R_src=pose2dsp_src.getRoationMatrix();   
   
   Eigen::Matrix3f R_ref_src=R_ref.transpose()*R_src;
   
   Eigen::Vector3f t_ref=pose2dsp_ref.getVec3f();
   Eigen::Vector3f t_src=pose2dsp_src.getVec3f();
    
   Eigen::Vector3f t_src_ref=R_ref.transpose()*(t_src-t_ref);
   
   Eigen::Quaternion<float> q(R_ref_src);
   
     Eigen::Vector3f v;
	//以下的处理非常重要，只适用于只存在偏航角的情况
	float a1=acos(q.w());
	float a2=-a1;
	if(abs(sin(a1)-q.z())<0.001f)
	{
	  v(2)=2*a1;
	}
	else if(abs(sin(a2)-q.z())<0.001f)
	{
	  v(2)=2*a2;
	  //cout<<v(2)<<endl;
	}
	else{
	std:: cout<<"四元数解析错误，该旋转轴不为z轴"<<std::endl;
	  exit(0);
	}

    v(1)=t_src_ref(1);
    v(0)=t_src_ref(0);
    pose2d pose(v(0),v(1),v(2));
    relativePose=pose;
    return pose;   
  
}


int RepeatabilityTest::getCorrespondingsNum()
{
  
  std::vector<std::pair<int, int>> groundkpPairs;
  int num=0;
  //PointCloud::Ptr tcp_kps;
  relativePose=computeRelativePose();
  
  std::vector<int> unassociatedSrcIndexes, unassociatedRefIndexes;
  std::vector<bool> ref_kps_bool(ref_kps.size(),false);
 PointCloud::Ptr rgs_kps_ptr(new PointCloud);
  transformpcd(src_kps.makeShared(),rgs_kps_ptr,relativePose.getVec3f(), relativePose.getRoationMatrix());
  
rgsteredsrc_kps=*rgs_kps_ptr;
  
  PointCloud refpcd=sensor::fromRangeData(Range_ref);
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  
  kdtree.setInputCloud(ref_kps.makeShared());
     int j=0;
  for(point3d kp:rgsteredsrc_kps)
  {  
     int n=1;
     std::vector<int> indexes;     
    std::vector<float> NKNSquareddistances;
    kdtree.nearestKSearch(kp,n,indexes,NKNSquareddistances);
    
    if(sqrt(NKNSquareddistances[0])<kp_association_dist_thres)
    {
      num++;
      ref_kps_bool[indexes[0]]=true;  
      groundkpPairs.push_back(std::make_pair(j, indexes[0]));
    }
    else{
      unassociatedSrcIndexes.push_back(j);
    }
    j++;
  }
  
  for(int i=0;i<ref_kps_bool.size();i++)
  {
    if(!ref_kps_bool[i])
    {
      unassociatedRefIndexes.emplace_back(i);
    }
  }  
  
  
  return num;  
}

int RepeatabilityTest::getLatentCorrespondingsNum(const std::vector<int> &unassociatedSrcIndexes, const std::vector<int> &unassociatedRefIndexes)
{ 
   
	  int latentNum=0, empty_supportRegion_num=0;
	  
	    PointCloud::Ptr registeredpcd(new PointCloud);
	    PointCloud refpcd=sensor::fromRangeData(Range_ref);
	    
	    PointCloud srcpcd=sensor::fromRangeData(Range_src);
	    transformpcd(srcpcd.makeShared(),registeredpcd,relativePose.getVec3f(), relativePose.getRoationMatrix());
	    
	    pcl::KdTreeFLANN<point3d> Kdtree_ref,Kdtree_src;
	    assert(!refpcd.empty());
	    assert(!registeredpcd->empty());
	    
	    
	     if(re_debug)
	     {
		  boost::shared_ptr< pcl::visualization::PCLVisualizer> viewer;
		  viewer=viz::twoVis(refpcd.makeShared(),registeredpcd);
		  
		  while(!viewer->wasStopped())
		  {
		    if(!viewer->wasStopped ())          viewer->spinOnce (100);
			boost::this_thread::sleep (boost::posix_time::microseconds (100000));
		  }
	     }
	    Kdtree_ref.setInputCloud(refpcd.makeShared());
	    Kdtree_src.setInputCloud(registeredpcd);
	    for(int unas_ref_kp_index:unassociatedRefIndexes)
	    {
	      std::vector<int> NKNpointIndexes;
	      std::vector<float> NkNsquaredDistances;
	      std::vector<float> ToallsquaredDistances;      
	      
	      //获取支持域
	      
	      std::vector<point3d> suportRegion;
	      
	    int index_ref_kp=  kp_kps_ref[unas_ref_kp_index].index;
	    suportRegion= getSuportRegionsfrompcd(index_ref_kp,refpcd.makeShared());
	    if(suportRegion.empty())
	    {
	      empty_supportRegion_num++;
	      continue;
	    }
	    
	      //assert(!suportRegion.empty());
	    for(point3d temp:suportRegion)
	    {
	      Kdtree_src.nearestKSearch(temp,1,NKNpointIndexes,NkNsquaredDistances);
	      ToallsquaredDistances.push_back(sqrt(NkNsquaredDistances[0]));
	      ReTest_DEBUG(sqrt(NkNsquaredDistances[0])<<std::endl);
	    }
	      double H_distance=std::accumulate(ToallsquaredDistances.begin(),ToallsquaredDistances.end(),0);
	      assert(!ToallsquaredDistances.empty());
	      H_distance/=ToallsquaredDistances.size();
	      ReTest_DEBUG("H_distance:"<<H_distance<<std::endl);
	      if(H_distance<=max_hausdorff_distance)
	      {
		latentNum++;
	      }
	    }
	    
	      for(auto unas_src_kp_index:unassociatedSrcIndexes)
	    {
	      std::vector<int> NKNpointIndexes;
	      std::vector<float> NkNsquaredDistances;
		std::vector<float> ToallsquaredDistances;      
	      std::vector<point3d> suportRegion;
		  int index_src_kp=  kp_kps_src[unas_src_kp_index].index;
		  
	    suportRegion= getSuportRegionsfrompcd(index_src_kp,registeredpcd);
	    if(suportRegion.empty())
	    {
	      empty_supportRegion_num++;
	      continue;
	    }
		//assert(!suportRegion.empty());
	    for(point3d temp:suportRegion)
	    {
	      Kdtree_ref.nearestKSearch(temp,1,NKNpointIndexes,NkNsquaredDistances);
	      ToallsquaredDistances.push_back(sqrt(NkNsquaredDistances[0]));
	      ReTest_DEBUG(sqrt(NkNsquaredDistances[0])<<std::endl);
		  assert(!ToallsquaredDistances.empty());
	    }
	    double H_distance=std::accumulate(ToallsquaredDistances.begin(),ToallsquaredDistances.end(),0.d);
	    H_distance/=ToallsquaredDistances.size();
	      ReTest_DEBUG("H_distance:"<<H_distance<<std::endl);
	      
	      if(H_distance<=max_hausdorff_distance)
	      {
		latentNum++;
	      }
	    }
	    
	      ReTest_DEBUG("the number of keypoint whose support region is empty: "<<empty_supportRegion_num<<std::endl);
	    
	    return latentNum;
   
}



double RepeatabilityTest::getRepeatabilityDetecor()
{
  if(!isdetected)
  {
    std::cout<<"please call detect function to detect the keypoints first"<<std::endl;
    exit(0);
  }
  groundKpPairs.clear();
  
  //std::vector<std::pair<int, int>> groundkpPairs;
  int num=0;
  //PointCloud::Ptr tcp_kps;
  relativePose=computeRelativePose();
  
  std::vector<int> unassociatedSrcIndexes, unassociatedRefIndexes;
  std::vector<bool> ref_kps_bool(ref_kps.size(),false);
 
  PointCloud::Ptr tem_rgs_src_kps(new PointCloud);
  transformpcd(src_kps.makeShared(),tem_rgs_src_kps,relativePose.getVec3f(), relativePose.getRoationMatrix());  
  
  if(re_debug)
  {
   boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
       viewer = viz::twoVis(tem_rgs_src_kps,ref_kps.makeShared());
	while (!viewer->wasStopped ())
	  {
		  if(!viewer->wasStopped ())          viewer->spinOnce (100);
          boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	  }
  }
  
  rgsteredsrc_kps=*tem_rgs_src_kps;
  
  
  
  PointCloud refpcd=sensor::fromRangeData(Range_ref);
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  
  assert(!ref_kps.empty());
  kdtree.setInputCloud(ref_kps.makeShared());
  
  
      int j=0;
  for(point3d kp:rgsteredsrc_kps)
  {
 
     int n=1;
     std::vector<int> indexes;
     
    std::vector<float> NKNSquareddistances;
    kdtree.nearestKSearch(kp,n,indexes,NKNSquareddistances);
    
    if(sqrt(NKNSquareddistances[0])<kp_association_dist_thres)
    {
      num++;
      ref_kps_bool[indexes[0]]=true;  
      groundKpPairs.push_back(std::make_pair(j, indexes[0]));
    }
    else{
      unassociatedSrcIndexes.push_back(j);
    }
    j++;
  }
  
  for(int i=0;i<ref_kps_bool.size();i++)
  {
    if(!ref_kps_bool[0])
    {
      unassociatedRefIndexes.emplace_back(i);
    }
  }
  
  ReTest_DEBUG("未被关联的特征点数目："<<unassociatedRefIndexes.size()+unassociatedSrcIndexes.size()<<std::endl);
  
  if(num==0)
  {
    return 0;
  }
  
  int latentNum=getLatentCorrespondingsNum(unassociatedSrcIndexes,unassociatedRefIndexes);    

  return  double(num)/double(latentNum+num);  
  
  
    
}



double RepeatabilityTest::getRepeatabilityDescriptos()
{
   
  if(groundKpPairs.empty())
  {
    std::cout<<" skip the data due to absence of the ground truth association  "<<std::endl;
    return double(0);
  }
  if(matchedKpPairs.empty())
  {
    return double(0);
  }
  int legNum=0;
  for(std::pair<int, int> gkppair:groundKpPairs)
  {
    for(std::pair<int,int> mkppair:matchedKpPairs)
    {
      if(gkppair.first==mkppair.first)
      {
	if(gkppair.second==mkppair.second)
	{
	  legNum++;
	}
      }
    }
  }
  
  return double(legNum)/double(matchedKpPairs.size());
  
}




void RepeatabilityTest::setDetectedTrue()
{
  isdetected=true;
}

void RepeatabilityTest::setMatchTrue()
{
   ismathched=true;
}


bool RepeatabilityTestDALKO::detect()
{
  
       // extractGeometryFeature gfs(Range_src),gfs1(Range_ref);      
	     //extractGeometryFeature gfs(Range_src),gfs1(Range_ref);    
        gfs.setInputRanges(Range_src);
	gfs.setRegionGrowRadius(0.4);
	src_kps.clear();
        src_kps= gfs.extractgfs("IFAKLO");
	kp_kps_src.clear();
	kp_kps_src=gfs.getKeypoints();
	
	src_kps_des=gfs.getGCdiscriptor();
	
	gfs.setInputRanges(Range_ref);
	//gfs1.setRegionGrowRadius(0.4);
	ref_kps.clear();
	//ref_kps=gfs1.extractgfs("IFAKLO");
	  ref_kps= gfs.extractgfs("IFAKLO");
	kp_kps_ref.clear();
	//kp_kps_ref=gfs1.getKeypoints();
	kp_kps_ref=gfs.getKeypoints();
        ref_kps_des=gfs.getGCdiscriptor(); 
	setDetectedTrue();	
	//assert(!src_kps.empty());assert(!ref_kps.empty());
	if(src_kps.empty()||ref_kps.empty())
	return false;
	else
       return true;
	
}

int  RepeatabilityTestDALKO::match()
{
  matchedKpPairs.clear();
  std::vector<std::tuple<int, int,double>> pairswithscores=  gfs.match(kp_kps_src,src_kps_des);
  for(auto pair:pairswithscores)
  {
    int ref_index, src_index;
    double score;
    std::tie(ref_index,src_index,score)=pair;
    //因为groudpairs中是src_index,ref_index
    matchedKpPairs.emplace_back(std::make_pair(src_index,ref_index));    
    

  }    
      ReTest_DEBUG("num of matching success: "<<matchedKpPairs.size()<<std::endl);
      if(re_debug)
      {
	
      }
  return matchedKpPairs.size();
}


bool RepeatabilityTestFALKO::detect()
{
        falkolib::FALKOExtractor fe;
	fe.setMinScoreTh(0);
	fe.setMinExtractionRange(0.5);
	fe.setMaxExtractionRange(30);
	fe.enableSubbeam(false);
	fe.setNMSRadius(0.2);
	fe.setNeighB(0.07);
	fe.setBRatio(3);//2.5
	fe.setGridSectors(36);//16            
    
       falkolib::LaserScan scan_ref=sensor_bridge::falkoscanfromRangeData(Range_ref);
	std::vector<falkolib::FALKO>  keypoints_ref;
	fe.extract(scan_ref, keypoints_ref);
	ReTest_DEBUG("keypoints_ref num:"<<keypoints_ref.size()<<std::endl);
	for(falkolib::FALKO kpi:keypoints_ref)
	{	  
	   geomeasurer::keypoint temkp;
	   temkp.index=kpi.index;	
	   kp_kps_ref.push_back(temkp);
	   ref_kps.push_back(ref_pcd.points[kpi.index]);	  
	}
 
 
	falkolib::LaserScan scan_src=sensor_bridge::falkoscanfromRangeData(Range_src);
        std::vector<falkolib::FALKO> keypoints_src;
	
        fe.extract(scan_src, keypoints_src);	
	ReTest_DEBUG("keypoints_src num:"<<keypoints_src.size()<<std::endl);
	for(falkolib::FALKO kpi:keypoints_src)
	{
	 geomeasurer::keypoint temkp;
	 temkp.index=kpi.index;	 
	  kp_kps_src.push_back(temkp);
	  src_kps.push_back(src_pcd.points[kpi.index]);
	}
	
       setDetectedTrue();
	if(src_kps.empty()||ref_kps.empty())
	{
	  return false;
	}
	else{
	  return true;
	}
}


int RepeatabilityTestFALKO::match()
{
    
}


bool RepeatabilityTestFLIRT::detect()
{
    CurvatureDetector *m_detectorCurvature = NULL;
     Detector* m_detector = NULL;
    unsigned int scale = 5, dmst = 2, window = 3, detectorType = 0, descriptorType = 0, distanceType = 2, strategy = 0;
    double baseSigma = 0.2, sigmaStep = 1.4, minPeak = 0.34, minPeakDistance = 0.001, acceptanceSigma = 0.1, success = 0.95, inlier = 0.4, matchingThreshold = 0.4;
    bool useMaxRange = false; 
      SimpleMinMaxPeakFinder *m_peakMinMax = new SimpleMinMaxPeakFinder(minPeak, minPeakDistance);    
     m_detectorCurvature = new CurvatureDetector(m_peakMinMax, scale, baseSigma, sigmaStep, dmst);
     m_detectorCurvature->setUseMaxRange(useMaxRange);
     m_detector = m_detectorCurvature;

     LaserReading lasereading_src=sensor_bridge::fromRangeData(Range_src);
     LaserReading lasereading_ref=sensor_bridge::fromRangeData(Range_ref);
     keypoints_ref.clear();keypoints_src.clear();
     //std::vector<InterestPoint*> keypoints_src,keypoints_ref;
     m_detector->detect(lasereading_src,keypoints_src);
     m_detector->detect(lasereading_ref,keypoints_ref);     
    src_kps=sensor_bridge::fromInterstPoints(keypoints_src);
    ref_kps=sensor_bridge::fromInterstPoints(keypoints_ref);
    for(auto src_kp:src_kps)
    {
      int index=Range_src.getIndexbyPoint(src_kp);
      geomeasurer::keypoint kp;
      kp.index=index;
      kp_kps_src.push_back(kp);      
    }
    
    for(auto ref_kp:ref_kps)
    {
      int index=Range_ref.getIndexbyPoint(ref_kp);
      geomeasurer::keypoint kp;
      kp.index=index;
      kp_kps_ref.push_back(kp);      
    }
    
        setDetectedTrue();
    if(src_kps.empty()||ref_kps.empty())
	{
	  return false;
	}
	else{
	  return true;
	}
    
}


int RepeatabilityTestFLIRT::match()
{

}


int RepeatabilityTestFLIRT::getLatentCorrespondingsNum(const std::vector< int >& unassociatedSrcIndexes, const std::vector< int >& unassociatedRefIndexes)
{
  int latentNum=0;
    PointCloud::Ptr registeredpcd(new PointCloud);
    PointCloud refpcd=sensor::fromRangeData(Range_ref);
    
    PointCloud srcpcd=sensor::fromRangeData(Range_src);
    transformpcd(srcpcd.makeShared(),registeredpcd,relativePose.getVec3f(), relativePose.getRoationMatrix());
    
    pcl::KdTreeFLANN<point3d> Kdtree_ref,Kdtree_src;
    assert(!refpcd.empty());
    assert(!registeredpcd->empty());
    
    
      if(re_debug)
      {
	  boost::shared_ptr< pcl::visualization::PCLVisualizer> viewer;
	  viewer=viz::twoVis(refpcd.makeShared(),registeredpcd);
	  
	  while(!viewer->wasStopped())
	  {
	    if(!viewer->wasStopped ())          viewer->spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	  }
      }
    
    Kdtree_ref.setInputCloud(refpcd.makeShared());
    Kdtree_src.setInputCloud(registeredpcd);
         int empty_supportRegion_num=0;
    for(int unas_ref_kp_index:unassociatedRefIndexes)
    {
 
      std::vector<int> NKNpointIndexes;
      std::vector<float> NkNsquaredDistances;
      std::vector<float> ToallsquaredDistances;      
      std::vector<point3d> suportRegion;

     // InterestPoint kp_tem=* (keypoints_ref[unas_ref_kp_index]);
    //  assert(!keypoints_ref.empty());
      std::vector<Point2D> suportRegion_2d=keypoints_ref[unas_ref_kp_index]->getSupport();
      for(Point2D p2d:suportRegion_2d)
      {
	point3d tem;
	tem.x=p2d.x;
	tem.y=p2d.y;
	tem.z=0.d;
	suportRegion.push_back(tem);
      }
     // int index_ref_kp=  kp_kps_ref[unas_ref_kp_index].index;
    // suportRegion= getSuportRegionsfrompcd(index_ref_kp,refpcd.makeShared());
    //  assert(!suportRegion.empty());
    if(suportRegion.empty())
    {
    
      empty_supportRegion_num++;
      continue;
    }
     for(point3d temp:suportRegion)
     {
      Kdtree_src.nearestKSearch(temp,1,NKNpointIndexes,NkNsquaredDistances);
      ToallsquaredDistances.push_back(sqrt(NkNsquaredDistances[0]));
      ReTest_DEBUG(sqrt(NkNsquaredDistances[0])<<std::endl);
     }
      double H_distance=std::accumulate(ToallsquaredDistances.begin(),ToallsquaredDistances.end(),0);
      assert(!ToallsquaredDistances.empty());
       H_distance/=ToallsquaredDistances.size();
       ReTest_DEBUG("H_distance:"<<H_distance<<std::endl);
      if(H_distance<=max_hausdorff_distance)
      {
	latentNum++;
      }
    }
    
      for(auto unas_src_kp_index:unassociatedSrcIndexes)
    {
      std::vector<int> NKNpointIndexes;
      std::vector<float> NkNsquaredDistances;
        std::vector<float> ToallsquaredDistances;      
      std::vector<point3d> suportRegion;
      
      //InterestPoint kp_tem=* keypoints_src[unas_src_kp_index];
      std::vector<Point2D> suportRegion_2d=keypoints_src[unas_src_kp_index]->getSupport();
      for(Point2D p2d:suportRegion_2d)
      {
	point3d tem;
	tem.x=p2d.x;
	tem.y=p2d.y;
	tem.z=0.d;
	suportRegion.push_back(tem);
      }
         //assert(!suportRegion.empty());
         if(suportRegion.empty())
	 {
	      empty_supportRegion_num++;
	   continue;
	}
     for(point3d temp:suportRegion)
     {
      Kdtree_ref.nearestKSearch(temp,1,NKNpointIndexes,NkNsquaredDistances);
      ToallsquaredDistances.push_back(sqrt(NkNsquaredDistances[0]));
      ReTest_DEBUG(sqrt(NkNsquaredDistances[0])<<std::endl);
           assert(!ToallsquaredDistances.empty());
     }
     double H_distance=std::accumulate(ToallsquaredDistances.begin(),ToallsquaredDistances.end(),0.d);
    H_distance/=ToallsquaredDistances.size();
       ReTest_DEBUG("H_distance:"<<H_distance<<std::endl);
      
      if(H_distance<=max_hausdorff_distance)
      {
	latentNum++;
      }
    }
    
    ReTest_DEBUG("The number of keypoints whose support region is empty:"<<empty_supportRegion_num<<std::endl);
    
    return latentNum;
}

double RepeatabilityTestFLIRT::getRepeatabilityDetecor()
{
if(!isdetected)
  {
    std::cout<<"please call detect function to detect the keypoints first"<<std::endl;
    exit(0);
  }
  groundKpPairs.clear();
  
  //std::vector<std::pair<int, int>> groundkpPairs;
  int num=0;
  //PointCloud::Ptr tcp_kps;
  relativePose=computeRelativePose();
  
  std::vector<int> unassociatedSrcIndexes, unassociatedRefIndexes;
  std::vector<bool> ref_kps_bool(ref_kps.size(),false);
 
  PointCloud::Ptr tem_rgs_src_kps(new PointCloud);
  transformpcd(src_kps.makeShared(),tem_rgs_src_kps,relativePose.getVec3f(), relativePose.getRoationMatrix());  
  
 
  if(re_debug)
  {
   boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
       viewer = viz::twoVis(tem_rgs_src_kps,ref_kps.makeShared());
	while (!viewer->wasStopped ())
	  {
		  if(!viewer->wasStopped ())          viewer->spinOnce (100);
          boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	  }
  }
  
  rgsteredsrc_kps=*tem_rgs_src_kps;  
  
  PointCloud refpcd=sensor::fromRangeData(Range_ref);
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  
  assert(!ref_kps.empty());
  kdtree.setInputCloud(ref_kps.makeShared());
  
  
    int j=0;
  for(point3d kp:rgsteredsrc_kps)
  {
 
     int n=1;
     std::vector<int> indexes;
     
    std::vector<float> NKNSquareddistances;
    kdtree.nearestKSearch(kp,n,indexes,NKNSquareddistances);
    
    if(sqrt(NKNSquareddistances[0])<kp_association_dist_thres)
    {
      num++;
      ref_kps_bool[indexes[0]]=true;  
      groundKpPairs.push_back(std::make_pair(j, indexes[0]));
    }
    else{
      unassociatedSrcIndexes.push_back(j);
    }
    j++;
  }
  
  for(int i=0;i<ref_kps_bool.size();i++)
  {
    if(!ref_kps_bool[0])
    {
      unassociatedRefIndexes.emplace_back(i);
    }
  }
  
  ReTest_DEBUG("未被关联的特征点数目："<<unassociatedRefIndexes.size()+unassociatedSrcIndexes.size()<<std::endl);
  
  if(num==0)
  return 0;
  int latentNum=getLatentCorrespondingsNum(unassociatedSrcIndexes,unassociatedRefIndexes);   
  
  return  double(num)/double(latentNum+num);  
}



}//geomeasurer


