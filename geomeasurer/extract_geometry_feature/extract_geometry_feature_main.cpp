#include "extractgeometryfeature.h"

#include "../viz/viz.hpp"
#include <cstring>
#include <opencv/highgui.h>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2//core/core.hpp>
#include <boost/graph/graph_concepts.hpp>
#include<ctime>
using namespace geomeasurer;
int main()
{ 

  cv::Mat img(1000,1000,CV_8UC1, cv::Scalar(255)); 
 

  sensor::rangeData ranges,ranges1;
  std::string filedir="../geomeasurer/data/dataflr4scan001";
  std::string filedir1="../geomeasurer/data/dataflr4scan003";
  
  ranges=io::fromFile(filedir);
  ranges1=io::fromFile(filedir1);
  extractGeometryFeature gfs(ranges);
  extractGeometryFeature gfs1(ranges1);
    gfs.setRegionGrowRadius(0.2);
   gfs1.setRegionGrowRadius(0.2);
   featurePointSet featurepoints1=gfs1.extractgfs("IFAKLO");
    std::time_t begin_t,end_t;
    begin_t=std::clock();
    featurePointSet featurePoints=gfs.extractgfs("IFAKLO");
    end_t=std::clock();
    std::cout<<"DALKO run time:"<<double(end_t-begin_t)/CLOCKS_PER_SEC<<std::endl;


    
    

    cv::Mat img1,img2;
    img1=viz::imshowPCDWithKeypoints("ifaklo", ranges,featurePoints);    
    
   img2=viz::imshowPCDWithKeypoints("p1",ranges1,featurepoints1);
     
    
     Discriptors GC=gfs.getGCdiscriptor();
     std::cout<<"GC:"<<std::endl;
    std::cout<<GC<<std::endl; 
     
     Discriptors GC1=gfs1.getGCdiscriptor();
     std::cout<<"GC1:"<<std::endl;
     std::cout<<GC1<<std::endl;
     
     geomeasurer::KeyPoints KPS1=gfs1.getKeypoints();
     std::vector<std::tuple<int,int, double> > pairs;
     std::cout<<"开始匹配:"<<std::endl;
    pairs= gfs.match(KPS1,GC1);
    std::vector<std::pair<point3d,point3d> > kppairs; 
    for(auto pair:pairs)
    {
      int index1, index2;
      double score;
      std::tie(index1,index2,score)=pair;
      point3d temp1=gfs.GetPoint3dfromIndex(index1);
      point3d temp2=gfs1.GetPoint3dfromIndex(index2);
      kppairs.push_back(std::make_pair(temp1,temp2));
      std::cout<<"index1:"<<index1<<"      "<<"index2:"<<index2<<"    "<<"score:"<<score<<std::endl;
    }
    
    viz::imshowMatch(img1,img2,kppairs);
     
     
     
     


 //viz::imshowPCDWithKeypoints("extractCornerFromRanges",ranges,featurePoints_rangeResponsed);  
  
  /*
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer);
 // viewer=map3d::twoVis(candidate_pcd.makeShared(),featurePoints.makeShared());
  viewer=map3d::twoVis(candidate_pcd.makeShared(),featurePoints_falko.makeShared());
  while(!viewer->wasStopped())
  {
    viewer->spinOnce();
    boost::this_thread::sleep(boost::posix_time::microseconds(10000));
  }
*/
}