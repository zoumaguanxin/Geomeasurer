/*
 * Copyright (c) 2017, <copyright holder> <email>
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

#include "extractgeometryfeature.h"
#include <boost/graph/graph_concepts.hpp>

namespace geomeasurer {
  
keypoint::keypoint()
{

}
keypoint::keypoint(const int & index_, const double& score_)
{
index=index_;
score=score_;
}

bool keypoint::compare(const keypoint& kp1, const keypoint& kp2)
{
    if(kp1.score>kp2.score)
     {
       return true;
    }
    else{
      return false;
    }     
}

  
extractGeometryFeature::extractGeometryFeature()
{
}
 
extractGeometryFeature::extractGeometryFeature(const PointCloud& pcd)
{
   candiate_pcd=pcd;
}

extractGeometryFeature::extractGeometryFeature(const sensor::rangeData& ranges_data)
{
  candiate_pcd=sensor::fromRangeData(ranges_data);
  ranges=ranges_data;
}

void extractGeometryFeature::setSlideWindowSizes(const int& slide_window_sizes_)
{
  slide_window_sizes=slide_window_sizes_;
}

void extractGeometryFeature::setRegionGrowRadius(const double& region_grow_radius_)
{
  region_grow_radius=region_grow_radius_;
}

void extractGeometryFeature::setClusterMinSize(const uint32_t& cluster_min_size_)
{
  cluster_min_size=cluster_min_size_;
}



std::vector< pcl::PointIndices > extractGeometryFeature::EuclideanClusterExtraction() const
{
  std::vector<pcl::PointIndices> ret;
  pcl::PointIndices cluster_indices;
  int indice=0;
  point3d bound_point;
  for(auto point:candiate_pcd.points)
  {
    
    if(cluster_indices.indices.empty())
    {
     bound_point=point;     
     cluster_indices.indices.emplace_back(indice);
    }
    else
    {
      double dist=math::pointDistance(bound_point,point);
      //std::cout<<"distance:"<<dist<<std::endl;
      bound_point=point;
          if(dist<region_grow_radius)
	  {
	    //std::cout<<"该类新增加一个点"<<std::endl;
	    cluster_indices.indices.emplace_back(indice);
	   }
	  else
	  {
	    std::cout<<"发现一个新的类,其尺寸为："<<cluster_indices.indices.size()<< std::endl;
	     ret.emplace_back(cluster_indices);
	     cluster_indices.indices.clear();
	     cluster_indices.indices.emplace_back(indice);
	  }	  
    }
    indice++;
  }
  ret.emplace_back(cluster_indices);
  return ret;
}



/**
 * @brief FAKLO
 */
featurePointSet extractGeometryFeature::extractFAKLO()
{
	featurePointSet ret_keypoints;
	falkolib::FALKOExtractor fe;
	fe.setMinScoreTh(0);
	fe.setMinExtractionRange(0.5);
	fe.setMaxExtractionRange(30);
	fe.enableSubbeam(false);
	fe.setNMSRadius(0.2);
	fe.setNeighB(0.07);
	fe.setBRatio(3);//2.5
	fe.setGridSectors(36);//16

	falkolib::LaserScan scan(ranges.angle_min,6*3.1419f/4, ranges.ranges.size());
	scan.fromRanges(ranges.ranges);
	std::vector<falkolib::FALKO> keypoints;
	fe.extract(scan, keypoints);
	for(falkolib::FALKO keypoint:keypoints)
	{
	  ret_keypoints.push_back(candiate_pcd.points[keypoint.index]);
	}
	return ret_keypoints;
}



void extractGeometryFeature::getNeiborhoodMember
(const int &numberofCenter2clusterboundary, const size_t& center_index, const double& radius, std::vector< point3d >& neigh, bool flag_Left)
{
     
     //根据邻域半径找到左右邻域内的点
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
	 growing_dl=math::pointDistance(candiate_pcd.points[center_index],candiate_pcd.points[center_index-j]);	 
	 if(growing_dl<radius)
	 {
	    neigh.push_back(candiate_pcd.points[center_index-j]);
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

std::tuple< double, double,double> extractGeometryFeature::jointEvaluate
(const point3d& center, const std::vector< point3d >& LeftNeigh, const std::vector< point3d >& rightNeigh)
{

    //size_t max_scale= std::max<size_t>(LeftNeigh.size(),rightNeigh.size());
  std::vector<double> response_Angle_ratios;
  std::vector<double> all_angle_ratios;
  double score=0;
  int count=0, Response_count=0;
  double response_area=0;

  for(auto left:LeftNeigh)
  {
    for(auto right:rightNeigh)
    {
      double area=computeAreaGivenEndpointCoordinates(center,left,right);
      double edg1,edg2, angleRatio;
    edg1=math::pointDistance(center,left);
    edg2=math::pointDistance(center,right);
    double area_min=getAreaThres(edg1,edg2);
    angleRatio=area/area_min;
    if(area>area_min)
    {
      Response_count++;
      response_area+=angleRatio;
      response_Angle_ratios.push_back(angleRatio);
    }
    score+=angleRatio; 
    all_angle_ratios.push_back(angleRatio);
    count++;
    }    
  }  
  
  //数据分布均值
  double mean=score/double(count);
  //理论分布均值
  double response_mean=response_area/double(Response_count);
  
  double error=0, error1=0, error2=0;
  double sigma=0;
  
  for(auto angle_Ratio:response_Angle_ratios)
  {
    double residual=angle_Ratio-response_mean;
     error+=residual*residual;
  }
    double variance_response=error/double(Response_count-1);
  
    for(auto angle_Ratio:all_angle_ratios)
  {
     double residual=angle_Ratio-mean;
      error1+=residual*residual;
  }
    double variance_all=error1/double(count-1);
    
    for(auto angle_Ratio:all_angle_ratios)
  {
    double residual=angle_Ratio-response_mean;
     error2+=residual*residual;
  } 
  double mutual_variance=error2/double(count-1);
 
  
  //计算相对熵
  double sigma_theroy=std::sqrt(variance_response);
  double sigma_data=std::sqrt(variance_all);
  double alpha_data=1/(double(std::sqrt(M_PI)*sigma_data));
  double alpha_theroy=alpha_data*(sigma_data/sigma_theroy);
 
  double D_P_Q=0;

  std::vector<double> P, Q;
  double sum_p=0,sum_q=0;
    for(auto angle_Ratio:all_angle_ratios)
  {
      //   std::cout<<"alpha_data:"<<alpha_data<<std::endl;
   //std::cout<<"alpha_theroy:"<<alpha_theroy<<std::endl;
    double residual=angle_Ratio-response_mean;
    double residual_data=angle_Ratio-mean;
   // std::cout<<"residual:"<<residual<<std::endl;
   //std::cout<<"residual_data:"<<residual_data<<std::endl;
    double qi=alpha_theroy*exp(-1/2.d*(residual*residual)/variance_response);
    double pi=alpha_data*exp(-1/2.d*(residual_data*residual_data)/variance_all);
   //std::cout<<"qi:"<<qi<<std::endl;
    sum_p+=pi;
    sum_q+=qi;
   //std::cout<<"pi:"<<qi<<std::endl;
    P.push_back(pi);
    Q.push_back(qi);
   //double d=(qi*log(qi/pi)+pi*log(pi/qi))/2;  
   // D_P_Q+=d;
  }
  

  for(int i=0;i<P.size();i++)
  {
      D_P_Q+=(P[i]/sum_p*log(P[i]*sum_q/(sum_p*Q[i]))+Q[i]/sum_q*log(Q[i]*sum_p/(sum_q*P[i])))/2;   
    // D_P_Q+=Q[i]/sum_q*log(Q[i]*sum_p/(sum_q*P[i]));
    // D_P_Q+= P[i]/sum_p*log(P[i]*sum_q/(sum_p*Q[i]));
  }

  // D_P_Q=sum_q;
  return std::make_tuple(response_mean,mutual_variance,D_P_Q);
  
}



featurePointSet extractGeometryFeature::extractCornerWithimprovedFAKLO(const ClusterIndices& cluster)
{
  featurePointSet ret;
  keypoints.clear();  
    std::vector<point3d> NeighL;
    std::vector<point3d> NeighR;  
  for(int i=0;i<cluster.indices.size();i++)
  {
    point3d point_i;

    point_i=candiate_pcd.points[cluster.indices[i]];
    if(i<min_point_num)
    {
      continue;
    }
    
    //计算左右邻域
    double Neigh_Radius=getNeiborhood(ranges.ranges[cluster.indices[i]]);
     std::cout<<"Neigh_R:"<<Neigh_Radius<<std::endl;
    double numofcenter2clusterboundary;
    numofcenter2clusterboundary=i;
    getNeiborhoodMember(numofcenter2clusterboundary, cluster.indices[i],Neigh_Radius, NeighL,true);
    if(NeighL.size()>=min_point_num)
    {
      numofcenter2clusterboundary=cluster.indices.size()-i-1;
      getNeiborhoodMember(numofcenter2clusterboundary,cluster.indices[i],Neigh_Radius, NeighR,false);
    }
    else{
      NeighL.clear();       
      continue;
    }
   if(NeighR.size()<min_point_num)
    {
     NeighL.clear(); 
     NeighR.clear();
    continue;
    } 
    

    //根据面积决定是否被选为特征点候选点
    point3d x_L=candiate_pcd.points[cluster.indices[i-NeighL.size()]];
    point3d x_R=candiate_pcd.points[cluster.indices[i+NeighR.size()]];

   //double area=computeAreaGivenEndpointCoordinates(point_i,x_L,x_R); 
     

   // double edg1=math::pointDistance(point_i, x_L);
   // double edg2=math::pointDistance(point_i, x_R);
    

    
    //****************************************
    //直接使用面积作为响应
   //****************************************
/*    
    area_min_size=getAreaThres(edg1,edg2);
    std::cout<<"area_min_size: "<<area_min_size<<std::endl;
   if(area>area_min_size)*/

    //*********************
     //下面为FAKLO使用的方法 
    //*********************
  /*
    //底边长度
    double BaseEdge=std::abs(math::pointDistance(x_L,x_R));
    //高
    double High_length=area/BaseEdge;    
     std::cout<<"底边:"<<BaseEdge<<std::endl;
     std::cout<<"高:"<<High_length<<std::endl;
     std::cout<<"Thres:"<<ri/b_ratio<<std::endl;
     //if(deterBaseEdge>=(ri/b_ratio)&&deterHigh>=(ri/b_ratio))*/

     std::pair<double,double> InvarianceAndRightAngleScore;
     
    InvarianceAndRightAngleScore=EvaulateInvarianceNeigh(point_i,NeighL,NeighR);
   if(InvarianceAndRightAngleScore.first>ratio_invariance)
    {
      //计算栅格分值
         double ScoreL=0, ScoreR=0,score=0,extra_reward=0;
	ScoreL=EvaulateNeighDivergence(point_i,NeighL);
	// std::cout<<ScoreL<<std::endl;
	ScoreR=EvaulateNeighDivergence(point_i,NeighR);     
	double DivergenceScore=ScoreL+ScoreR;  
	// double AngleInvarianceScore=EvaulateAngleQuality(point_i,NeighL,NeighR);
	//score=InvarianceAndRightAngleScore.second;
	//score=AngleInvarianceScore;
	// score=gainClosingRightAngle*RightAnglescore;
	 //score=DivergenceScore;
	//score=AngleInvarianceScore+gainClosingRightAngle*RightAnglescore+DivergenceScore;
	double RightAnglescore,Variance;
	double responseScore;
      std::tie(RightAnglescore,Variance,responseScore) =jointEvaluate(point_i,NeighL,NeighR);
      // score=1000+0.8*RightAnglescoreAndVariance.first-0*RightAnglescoreAndVariance.second;
      // score=responseScore*RightAnglescore/std::sqrt(Variance);
       double expect=1/sin(min_angle/180.d*M_PI);
      // score=1000-responseScore;
      score=exp((expect-RightAnglescore)*(expect-RightAnglescore)*Variance);
      std::cout<<"score:"<<score<<std::endl;
      keypoints.emplace_back(keypoint(cluster.indices[i], score));
    }    
   NeighL.clear();
   NeighR.clear();
  }
  
  //如果不判断，当为空的情况下，迭代器就不会生效，这是对迭代器进行取值就会出错。
    if(!keypoints.empty())
    {
     auto kp_with_maxvalue=std::max_element(keypoints.begin(),keypoints.end(),keypoint::compare);
     keypoint temkp=*kp_with_maxvalue;
    double max_score=temkp.score;
	
	for(auto iter=keypoints.begin();iter!=keypoints.end();++iter)
	{
	  double tem=(*iter).score;
	  (*iter).score=tem;	 
	}
    }
 
  //孤立点抑制
 alonePointSupress();  
 //非极大值抑制 
 return NonMaxSupress();
 return fromKeypoints();
 
}


//TODO
featurePointSet extractGeometryFeature::extractFLIRT()
{
// PeakFinder * pf;
// RangeDetector range_detector(pf);
// std::vector<double> phl, rho;
// for(int i=0;i<ranges.ranges.size();i++)
// {
//   phl.emplace_back(ranges.angle_min+i*ranges.angle_increment);
//   rho.emplace_back(ranges.ranges[i]);
// }
// LaserReading Laser_reading(phl,rho);
// std::vector<InterestPoint*> kps;
// range_detector.detect(Laser_reading,kps);
}


/*
discriptors extractGeometryFeature::getGCdiscriptor(const point3d &center)
{
  getNeiborhoodMember();

}*/






//主要提取特征的主函数

featurePointSet extractGeometryFeature::extractgfs(std::string T)
{  
  featurePointSet ret;
   if(T== "FAKLO")
     {
         ret=extractFAKLO();
	 return ret;
     }
  //首先对点云进行聚类
  //不能使用大于阈值的特征点响应方法，要使用NMS的方法，每一个点都有一个响应值，在一个区域里面挑出一个最大的
  //对于二维首先统计出稳定的线点，然后对这点使用区域生成法恢复直线，然后对所有点进行聚类
  //由于初始的种子点可能在一个类中，所以，当种子被聚类到某一类中时，这个种子就会被删除点
  //然后再在每一个类中找到角点 
  std::cout<<"点云尺寸："<<candiate_pcd.size()<<std::endl;
  std::vector<pcl::PointIndices> clustersWithindices;
  if(IsClustering)
  {
   clustersWithindices=EuclideanClusterExtraction();  
  }
  else
  {
    pcl::PointIndices tempCluster;
    for(int i=0;i<ranges.ranges.size();i++)
    {
      tempCluster.indices.emplace_back(i);
    }
    clustersWithindices.emplace_back(tempCluster);
  }
  std::cout<<"聚类个数："<<clustersWithindices.size()<<std::endl;
  for(pcl::PointIndices cluster: clustersWithindices)
  {    
    if( T=="IFAKLO")
   {
         if(cluster.indices.size()>cluster_min_size)
	 {
           ret+=extractCornerWithimprovedFAKLO(cluster);
	 }
   }
    else if( T=="AT")
    {
        ret+=extractCornerWithAreaTsensor(cluster);
    }
    else
    {
       std::cout<<"please set feature type you would like to chose from FAKLO, IFAKLO, AT"<<std::endl;
    }
 } 
    return ret;
}



featurePointSet extractGeometryFeature::extractCornerWithAreaTsensor(const ClusterIndices &cluster)
{
    featurePointSet ret;
    keypoints.clear();    
  /*
   * 首先两个端点，用它构建面积张量, 在里面滑动我们的窗口，窗口的点选为端点。求出这个窗口内，在当前窗口形态下的响应值
   * 这样应该就很好检测直线特征点
   * 相对于最小least square的好处在于不用计算法向量，法向量十分容易受到噪声干扰
   */
  std::vector<std::pair<int,double> > responses;
  
  bool line_flag=false;
  int step=1;
  double rep,rep1,rep2;
  Eigen::Matrix3d S;
  S.col(2)<<1,1,1;
  if(cluster.indices.size()>=cluster_min_size)
  {      
    for(int n=0;n<cluster.indices.size()-slide_window_sizes-1;   n+=step)
    {
     std::cout<<"点在类中位置："<<n<<std::endl;      
      if(line_flag==false)
      { 
	 std::vector<point3d> Neigh;	 
        double growingR=math::pointDistance(candiate_pcd.points[cluster.indices[n]],candiate_pcd.points[cluster.indices[n+1]]);
	std::cout<<"growingR:"<<growingR<<std::endl;
	       int j=1;
	       ri=getNeiborhood(ranges.ranges[cluster.indices[n]]);
	   while(growingR<=ri&&(n+j)<cluster.indices.size())
	  {
	      Neigh.push_back(candiate_pcd.points[cluster.indices[n+j]]);
	      growingR=math::pointDistance(candiate_pcd.points[cluster.indices[n]],candiate_pcd.points[cluster.indices[n+j]]);
	      j++;
	   }
         slide_window_sizes=Neigh.size();
	 std::cout<<"slide_window_sizes:" <<slide_window_sizes<<std::endl;
	 Neigh.clear();
	 if(slide_window_sizes<min_point_num)
	 {
	   step=1;
	   continue;
	}
	  S(0,0)=candiate_pcd.points[cluster.indices[n]].x; 
	  S(0,1)=candiate_pcd.points[cluster.indices[n]].y;
	  S(1,0)=candiate_pcd.points[cluster.indices[n+slide_window_sizes]].x; 
	  S(1,1)=candiate_pcd.points[cluster.indices[n+slide_window_sizes]].y;
	  for(int j=0;j<slide_window_sizes;j++)
	  {
	    S(2,0)=candiate_pcd.points[cluster.indices[n+j]].x;
	    S(2,1)=candiate_pcd.points[cluster.indices[n+j]].y;
	      rep=std::abs(S.determinant());
              std::cout<<"case0 当前面积："<<rep<<std::endl;
	      if(rep>area_min_size)
		{
		  std::cout<<"case1 当前面积："<<rep<<std::endl;
		  line_flag=false;
		  step=1;
		 break;
		}
		else
		{
		  line_flag=true;
		  step++;
		}
	  } //for(int j=0;j<slide_window_sizes;j++)
      }
      else
      {
        S(2,0)=candiate_pcd.points[cluster.indices[n]].x;
        S(2,1)=candiate_pcd.points[cluster.indices[n]].y;

	rep=std::abs(S.determinant());
	
	if(rep>area_min_size)
	    {
	    std::cout<<"case2 当前面积："<<rep<<std::endl;
	    line_flag=false;
	    S(2,0)=candiate_pcd.points[cluster.indices[n+1]].x;
	    S(2,1)=candiate_pcd.points[cluster.indices[n+1]].y;
	   rep1=std::abs(S.determinant());
	    S(2,0)=candiate_pcd.points[cluster.indices[n+2]].x;
	    S(2,1)=candiate_pcd.points[cluster.indices[n+2]].y;
	     rep2=std::abs(S.determinant());	
	   if(rep1>area_min_size&&rep2>area_min_size)
	      keypoints.emplace_back(keypoint(cluster.indices[n-1],rep));
	     break;
	    }
	    else
	    {
	      step=1;
	     }
      }     
    }//for     
  }//if   
  //return fromKeypoints();
  return ret=NonMaxSupress();
}




featurePointSet extractGeometryFeature::NonMaxSupress()
{
  
  
  //仍然有待改进，也最好是先计算索引窗口范围，在这个索引窗口内直接抑制。同时，对于打分十分接近的，直接选用打分接近的结构上在中间的点，就不要在抑制了
  //先找出最大值，然后找到在该点抑制半径范围内的点，将这些点去除
  //直到容器为空
  KeyPoints nmsed_keypoints;
    featurePointSet ret;
  if(keypoints.empty())
  {
        return ret;      
  }
  
   /*
   std::cout<<"检测到特征点个数"<<keypoints.size()<<std::endl;     
   for(auto kp:keypoints)
   {
     std::cout<<"index:"<<kp.index<<" "<<"score:"<<kp.score<<std::endl;
  }
  */
   double max_score=0;
   bool first=true;
  while(!keypoints.empty())
  {
     KeyPoints tem_keypoints;
     auto kp_with_maxvalue=std::max_element(keypoints.begin(),keypoints.end(),keypoint::compare);
     keypoint temkp=*kp_with_maxvalue;
     if(first)
     {
       max_score=temkp.score;
    }
    nmsed_keypoints.emplace_back(temkp);
    keypoints.erase(kp_with_maxvalue);
    if(keypoints.empty())
    {
      break;
    }
    //注意，当中间元素被删除之后，容器后面的迭代器就会失效，所以不能直接在以下程序中使用keypoints.erase(it)。
    //为防止这种情况发生，加入一个临时容器    
    for(KeyPoints::iterator it=keypoints.begin();it!=keypoints.end();++it)
    {
      double dist=math::pointDistance(candiate_pcd.points[temkp.index],candiate_pcd.points[(*it).index]);
      std::cout<<"dist:"<<dist<<std::endl;
        if(dist>NMS_radius)
      {
	tem_keypoints.emplace_back(*it);
      }
    }   
   keypoints.clear();
   if(!tem_keypoints.empty())
   {
    keypoints.assign(tem_keypoints.begin(),tem_keypoints.end());
   }
   else
   {
     break;
  }    
  } 
  keypoints.clear();
  keypoints.assign(nmsed_keypoints.begin(),nmsed_keypoints.end());
  
  for(keypoint kp:keypoints)
  {
    if(kp.score>=minValPercent*max_score)
    {
      ret.push_back(candiate_pcd.points[kp.index]);
      Allkeypoints.push_back(kp);
    }
  }
  //keypoints.clear();
  std::cout<<"NMS后特征点个数:"<<ret.size()<<std::endl;
  return ret;  
 }  
 
 
featurePointSet extractGeometryFeature::alonePointSupress()
{
  KeyPoints tem_keypoints;
  for(auto kpi:keypoints)
  {
    int count=0;
    for(auto kpj:keypoints)
    {
      double dist=math::pointDistance(candiate_pcd.points[kpi.index],candiate_pcd.points[kpj.index]);
      if(dist<=alone_radius)
      {
	count++;
      }
      if(count>=alone)
      {
	tem_keypoints.emplace_back(kpi);
	//这个break很重要，不然太消耗时间
	break;
      }
    }
  }
  keypoints.clear();
  keypoints.assign(tem_keypoints.begin(),tem_keypoints.end());
  return fromKeypoints();
}

 
 
 //把特征点用点云输出
featurePointSet extractGeometryFeature::fromKeypoints()
{
  featurePointSet kp_PCD;
     for(auto kp:keypoints)
     {
       kp_PCD.push_back(candiate_pcd.points[kp.index]);
    }
    return kp_PCD;
}

featurePointSet extractGeometryFeature::fromAllkeypoints()
{
    keypointPcd.clear();
     for(auto kp:Allkeypoints)
     {
      keypointPcd.push_back(candiate_pcd.points[kp.index]);
    }
    return keypointPcd;
}


 
float extractGeometryFeature::getNeiborhood(const double& range)
{  
  ri=a*exp(b*range);
  return ri;
}

double extractGeometryFeature::getAreaThres()
{
  return 1/2.d*ri*ri*sin(min_angle/180.d*M_PI);
}


double extractGeometryFeature::getAreaThres(double edge1, double edge2)
{
  return 1/2.d*edge1*edge2*sin(min_angle/180.d*M_PI);
}

double extractGeometryFeature::extra_reward_from(const double& area,const double& area_min)
{
   return gainClosingRightAngle*area/area_min;
}

std::vector< int > extractGeometryFeature::computeNeighPlorDistribution(const point3d& center, const std::vector< point3d >& singleSideNeigh)
{
 std::vector<int> V_sector;
      double scoreSingleSide=0;
       int count=0;
      for(auto point:singleSideNeigh)
      {
	int SecNum=std::floor(gridSectorsNum/(2*M_PI)*std::atan2(point.y-center.y, point.x-center.x));
	if(SecNum<0)
	{
	  SecNum=gridSectorsNum+SecNum;
	}	
	V_sector.push_back(SecNum);
      }
      return V_sector;
}

double extractGeometryFeature::EvaulateAngleQuality(const point3d& center, const std::vector< point3d >& LeftNeigh, const std::vector< point3d >& rightNeigh)
{
  std::vector<int> V_left,V_Right, V_dist;
  V_left=computeNeighPlorDistribution(center,LeftNeigh);
  V_Right=computeNeighPlorDistribution(center,rightNeigh);
  int sum_dist=0,count=0,dist=0;
  for(auto left:V_left)
  {
    for(auto right:V_Right)
    {
     count++;      
     dist= std::abs((abs(left-right)+gridSectorsNum/2)%gridSectorsNum-gridSectorsNum/2);
     sum_dist+=dist;
     V_dist.emplace_back(dist);
    }
  }
  double mean_dist=sum_dist/double(count);
  double error=0,residual=0;
 for(auto dist:V_dist)
 {
   residual=dist-mean_dist;
   error+=residual*residual;
}
double variance=error/double(count-1); 
return variance;
}


double extractGeometryFeature::EvaulateNeighDivergence(const point3d &center, const  std::vector< point3d > &singleSideNeigh)
{
      std::vector<int> V_sector;
      double scoreSingleSide=0;
       int count=0;
      for(auto point:singleSideNeigh)
      {
	int SecNum=std::floor(gridSectorsNum/(2*M_PI)*std::atan2(point.y-center.y, point.x-center.x));
	if(SecNum<0)
	{
	  SecNum=gridSectorsNum+SecNum;
	}
	if(V_sector.size()>0)
	{
	    for(auto Sec_i:V_sector)
	    {
	      count++;
	      scoreSingleSide+=std::abs((abs(SecNum-Sec_i)+gridSectorsNum/2)%gridSectorsNum-gridSectorsNum/2);
	    }
	  }
	V_sector.push_back(SecNum);
      }
     //scoreSingleSide=scoreSingleSide/count;
     V_sector.clear();
      return scoreSingleSide;
}


std::pair<double,double> extractGeometryFeature::EvaulateInvarianceNeigh(const point3d& center, const std::vector< point3d >& LeftNeigh, const std::vector< point3d >& rightNeigh)
{
/*
  size_t max_scale= std::max<size_t>(LeftNeigh.size(),rightNeigh.size());
  std::vector<double> sin_Angle_ratios;
  double score=0;
  int Response_count=0;
  for(int i=0;i<max_scale;i++)
  {
    double area=computeAreaGivenEndpointCoordinates(center,LeftNeigh[i],rightNeigh[i]);
    double edg1,edg2, angleRatio;
    edg1=math::pointDistance(center,LeftNeigh[i]);
    edg2=math::pointDistance(center,rightNeigh[i]);
    double area_min=getAreaThres(edg1,edg2);
    angleRatio=area/area_min;
    score+=angleRatio;
    if(area>area_min) Response_count++;
    }
    return std::make_pair(Response_count/double(max_scale),score/double(max_scale));
    */

  double score=0;
  int count=0, Response_count=0;
  double response_area=0;
  for(auto left:LeftNeigh)
  {
    for(auto right:rightNeigh)
    {
      double area=computeAreaGivenEndpointCoordinates(center,left,right);
      double edg1,edg2, angleRatio;
    edg1=math::pointDistance(center,left);
    edg2=math::pointDistance(center,right);
    double area_min=getAreaThres(edg1,edg2);
    angleRatio=area/area_min;
    if(area>area_min)
    {
      Response_count++;
    }
    count++;
    }    
  }   
 return std::make_pair(Response_count/double(count),score/double(count));

}


double extractGeometryFeature::computeAreaGivenEndpointCoordinates(const point3d& A1, const point3d& A2, const point3d& A3)
{
    Eigen::Matrix3f S;
    S.col(2)<<1,1,1;
    S(0,0)=A1.x;    S(0,1)=A1.y;
    S(1,0)=A2.x;    S(1,1)=A2.y;
    S(2,0)=A3.x;    S(2,1)=A3.y;
    //面积
    double area=1/2.d*std::abs(S.determinant());
    return area;
}


Discriptors extractGeometryFeature::getGCdiscriptor()
{
  
    fromAllkeypoints();
    GCdiscriptors.resize(dscpSectorsNum,Allkeypoints.size()); 
    std::cout<<GCdiscriptors.rows()<<std::endl;
    std::cout<<GCdiscriptors.cols()<<std::endl;
    GCdiscriptors.setZero();
         int j=0,i=0;
    for(keypoint kp:Allkeypoints)
    {      
    point3d current_point=candiate_pcd.points[kp.index];    
   
    int Cornerorientation;
    if(!keypoints_based_method)
    { Cornerorientation=getCornerOrientationfromNeigh(kp.index);}
    else {Cornerorientation=getCornerOrientationBasedonKeypoints(kp.index);}  

    //计算特征点在栅格内的分布
    for(keypoint kpj:Allkeypoints)
      {
      if(kpj.index==kp.index)
      {
	continue;
      }
      point3d kppoint=candiate_pcd.points[kpj.index];
       int kp_sector_num;
       kp_sector_num=floor(dscpSectorsNum/(2*M_PI)*atan2(kppoint.y-current_point.y,kppoint.x-current_point.x));
       if(kp_sector_num<0)
       {
	 kp_sector_num+=dscpSectorsNum;
      }
      int diff=kp_sector_num-Cornerorientation;
      if(diff<0)
      {
	kp_sector_num=dscpSectorsNum+diff;
      }
      else{
	kp_sector_num=diff;
      }
       double dist=math::pointDistance(kppoint,current_point);
       if(GCdiscriptors(kp_sector_num,i)==0)
       GCdiscriptors(kp_sector_num,i)=log(dist);
       else if (GCdiscriptors(kp_sector_num,i)>log(dist))
       {
	    GCdiscriptors(kp_sector_num,i)=log(dist);
       }      
    }
     i++; 
    }
    return GCdiscriptors;
}

int extractGeometryFeature::getCornerOrientationfromNeigh(const int& index)
{
    double r=getNeiborhood(ranges.ranges[index]);
      std::vector<point3d> NeighL,NeighR;
      point3d current_point=candiate_pcd.points[index];
      getNeiborhoodMember(index,index,r,NeighL,true);
      getNeiborhoodMember(ranges.ranges.size()-index-1,index,r,NeighR, false);
      
      point3d centriod, centriod_left, centriod_right;
      centriod.x=0;centriod.y=0;centriod.z=0;
      centriod_left.x=0;centriod_left.y=0;centriod_left.z=0;
      centriod_right.x=0;centriod_right.y=0;centriod_right.z=0;

      //计算左右边各自的栅格分布
      double left_edge=0, right_edge=0;
      for(point3d temp:NeighL)
      {
	centriod_left.x+=temp.x;
	centriod_left.y+=temp.y;
	centriod_left.z+=temp.z;
      }
      centriod_left.x/=NeighL.size();      
      centriod_left.y/=NeighL.size();
      int Orientation_l=floor(dscpSectorsNum/(2*M_PI)*atan2(centriod_left.y-current_point.y,centriod_left.x-current_point.x));
      if(Orientation_l<0)
      {
	Orientation_l+=dscpSectorsNum;
      }
      
      for(point3d temp:NeighR)
      {
	centriod_right.x+=temp.x;
	centriod_right.y+=temp.y;
	centriod_right.z+=temp.z;
      }
      centriod_right.x/=NeighR.size();
      centriod_right.y/=NeighR.size();
      int Orientation_r=floor(dscpSectorsNum/(2*M_PI)*atan2(centriod_right.y-current_point.y,centriod_right.x-current_point.x));
      if(Orientation_r<0)
      {
	Orientation_r+=dscpSectorsNum;
      }    
     
      int Cornerorientation=1/2*(Orientation_l+Orientation_r);
      if ((abs(Orientation_l-Cornerorientation)+abs(Orientation_r-Cornerorientation))>(dscpSectorsNum/2.d))
      {
         Cornerorientation=(Cornerorientation+dscpSectorsNum/2)%dscpSectorsNum;
      }
      return Cornerorientation;
}


int extractGeometryFeature::getCornerOrientationBasedonKeypoints(const int & index)
{  
      int left_endpoint_index,right_endpoint_index;
      point3d current_point=candiate_pcd.points[index];
      double dist_min, dist_second;
      bool first=true;
     if(!isCreatedKdtree)
     { 
      kdtree_keypoints.setInputCloud(keypointPcd.makeShared());
      isCreatedKdtree=true;
     }
      
       size_t K=3;
       std:: vector<int> pointIdxNKNSearch(K);//用来存放搜索到的点的index
       std::vector<float> pointNKNSquaredDistance(K);//存放搜索到的点到当前的欧式距离
       kdtree_keypoints.nearestKSearch(current_point,K,pointIdxNKNSearch,pointNKNSquaredDistance);
       left_endpoint_index=pointIdxNKNSearch[1];
       right_endpoint_index=pointIdxNKNSearch[2];    
      point3d left_endpoint=keypointPcd.points[left_endpoint_index];
      point3d right_endpoint=keypointPcd.points[right_endpoint_index];
      point3d centriod;
      centriod.x=(left_endpoint.x+right_endpoint.x+current_point.x)/3;
      centriod.y=(left_endpoint.y+right_endpoint.y+current_point.y)/3;      
     int Cornerorientation=floor(dscpSectorsNum/(2*M_PI)*atan2(centriod.y-current_point.y,centriod.x-current_point.x));
      if (Cornerorientation<0)
      {
         Cornerorientation+=dscpSectorsNum;
      }
      return Cornerorientation;
}



std::vector<std::tuple< int, int, double > > extractGeometryFeature::match(const KeyPoints& kps, const Discriptors& GCS)
{ 
     int i=0;
     std::vector<std::tuple<int,int,double>> pairs;
    assert(GCS.cols()>0&&GCS.rows()>0);
    for(keypoint kpi:Allkeypoints)
    {double max_score=0, second_score=0;
      int max_index;
     int  j=0;
      for(keypoint kpj:kps)
      {	 
	double score=0;
	for(int k=0;k<GCS.rows();k++)
	{
	  if(GCS(k,j)>0&&GCdiscriptors(k,i)>0)
	  {
	  double dist=abs(GCS(k,j)-GCdiscriptors(k,i));
	  if(abs(dist)<=match_dist_thres)
	  {
	    score+=20.d;
	  }
	  }
	}
	 if(score>=max_score)
	 {
	second_score=max_score;
	max_score=score;
	max_index=kpj.index;
	 }	
	j++;
      }
      if((max_score/second_score)>=distinct_ratio&&(max_score>=match_score_min))
      {
	pairs.push_back(std::make_tuple(kpi.index,max_index,max_score));
      }
      i++;
    }    
    return pairs;
}


/**
 * 特征点，还有有一个特性也很重要，就是明显区别于其他点。这样可以减少在匹配中产生模糊性，可以想象，如果没有
 * 使用面积张量来检测特征点
 */
featurePointSet extractGeometryFeature::extractfromTensorField(const ClusterIndices & cluster)
{
    featurePointSet ret;
    keypoints.clear();    
  /*
   * 首先两个端点，用它构建面积张量, 在里面滑动我们的窗口，窗口的点选为端点。求出这个窗口内，在当前窗口形态下的响应值
   * 这样应该就很好检测直线特征点
   * 相对于最小least square的好处在于不用计算法向量，法向量十分容易受到噪声干扰
   */
  std::vector<std::pair<int,double> > responses;
  
  if(cluster.indices.size()>=cluster_min_size)
  {
    for(int n=0;n<cluster.indices.size()-slide_window_sizes;n++)
    {
      Eigen::Matrix3d S;
      S.col(2)<<1,1,1;
      S(0,0)=candiate_pcd.points[cluster.indices[n]].x; 
      S(0,1)=candiate_pcd.points[cluster.indices[n]].y;
      S(1,0)=candiate_pcd.points[cluster.indices[n+slide_window_sizes]].x; 
      S(1,1)=candiate_pcd.points[cluster.indices[n+slide_window_sizes]].y;
      
      for(int j=0;j<slide_window_sizes;j++)
      {
	S(2,0)=candiate_pcd.points[n+j].x;
        S(2,1)=candiate_pcd.points[n+j].y;
	  double rep=std::abs(S.determinant());
	  if(rep>area_min_size)
	    {	    
	      responses.push_back(std::make_pair(cluster.indices[n+j],rep));
	    }
	    else
	    {
	      responses.push_back(std::make_pair(cluster.indices[n+j],0.d));
	    }
      }
    }
  }
  //创建一个积分图像  
  std::vector<double> integral_response;
  double rep_integ=0;
  for(auto response:responses)
  {   
    rep_integ+=response.second;
    integral_response.emplace_back(rep_integ);
    std::cout<<response.second<<" ";
  }
   std::cout<<std::endl; 
   
   for(auto response:responses)
   {
     keypoints.emplace_back(keypoint(response.first, response.second));
  }
   
   //局部最大值
   for(int j=0;j<responses.size();j++)
   {
     if(j>1&&j<responses.size()-2)
     {
       if((responses[j].second>responses[j-1].second&&responses[j].second>responses[j+1].second)
	 ||(responses[j].second<responses[j-1].second&&responses[j].second<responses[j+1].second))
       {
	double variance=3*responses[j].second-(integral_response[j+1]-integral_response[j-1]);
	std::cout<<variance<<std::endl;
	keypoints.emplace_back(keypoint(responses[j].first, responses[j].second));
       }
    }
  }
  //return ret=fromKeypoints();
  return ret=NonMaxSupress();
}


KeyPoints extractGeometryFeature::getKeypoints()
{
   return Allkeypoints;
}

point3d extractGeometryFeature::GetPoint3dfromIndex(const int & pointIndex)
{
  return candiate_pcd.points[pointIndex];
}


 
}