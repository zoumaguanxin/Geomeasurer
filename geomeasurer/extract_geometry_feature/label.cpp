#include "extractgeometryfeature.h"

#include "../viz/viz.h"


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
  
   sensor::RangesCorrespondingtoposes rcps;
   std::string logfiledir="../geomeasurer/data/mit-cscail.log";
   
   rcps=io::fromlogfile(logfiledir);
   

    std::ofstream file_w;
  // file_w.open("../geomeasurer/data/groundtruth_intel", std::ios_base::out);
 
  for(int iter=0;iter<60;iter++)
  {
   
  sensor::rangeData ranges,ranges1;  
  ranges=rcps[iter].second;

  std::cout<<ranges.angle_increment<<" "<<ranges.angle_min<<" "<<ranges.angle_max<<" "<<ranges.maxRange<<std::endl;  
  std::cout<<ranges.ranges.size()<<std::endl;  
  for(auto range:ranges.ranges)
  {
    std::cout<<range<<" ";
  }  
  


  
    std::cout<<std::endl;
    extractGeometryFeature gfs(ranges);

    gfs.setRegionGrowRadius(0.4);

    std::time_t begin_t,end_t;
    begin_t=std::clock();
    featurePointSet featurePoints=gfs.extractgfs("IFAKLO");
    end_t=std::clock();
    std::cout<<"DALKO run time:"<<double(end_t-begin_t)/CLOCKS_PER_SEC<<std::endl;
    
    viz::imshowPCDWithKeypoints("dalko",ranges,featurePoints);

/*
    if (file_w.good())
    {
      int x;
      std::cout<<"请输入预期的特征点个数："<<std::endl;
      std::cin>>x;
      file_w<<x<<" ";      
    }
    else{
     exit(0);
    }
    */
   
    }
    

     
   
}