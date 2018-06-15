#include "../extract_geometry_feature/extractgeometryfeature.h"

#include "../viz/viz.h"

#include "../io/scan_reader.h"
#include <algorithm>
#include<numeric>

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
  std::vector<int> groundth;
  
  std::string filedir="../geomeasurer/data/groundtruth_fr079";
  std::string logfiledir="../geomeasurer/data/fr079.log";
  groundth=io::groudtruthfromfile(filedir);
  
  const int num=groundth.size();
  
  sensor::RangesCorrespondingtoposes   rcps=io::fromlogfile(logfiledir);
  
    sensor::rangeData ranges,ranges1;  
  
    int expectedNumkeypoints=std::accumulate(groundth.begin(),groundth.end(),0);
    int NumofdetectedKeypoints=0;    
    int NumFALKO=0;
    
  for(int iter=0;iter<num;iter++)
  {
   
  sensor::rangeData ranges;  
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
     NumofdetectedKeypoints+=featurePoints.size();
     
     
     
     
        falkolib::FALKOExtractor fe;
	fe.setMinScoreTh(0);
	fe.setMinExtractionRange(0.5);
	fe.setMaxExtractionRange(30);
	fe.enableSubbeam(false);
	fe.setNMSRadius(0.2);
	fe.setNeighB(0.07);
	fe.setBRatio(3);//2.5
	fe.setGridSectors(36);//16
        double angle_range=ranges.angle_max-ranges.angle_min;
	falkolib::LaserScan scan(ranges.angle_min,angle_range, ranges.ranges.size());
	scan.fromRanges(ranges.ranges);
	std::vector<falkolib::FALKO> keypoints;
	fe.extract(scan, keypoints);
     
     NumFALKO+=keypoints.size();
     
    }
    
  
  double DALKO_falseNegR=double(expectedNumkeypoints-NumofdetectedKeypoints)/double(expectedNumkeypoints);
  double FALKO_falseNegR=double(expectedNumkeypoints-NumFALKO)/double(expectedNumkeypoints);
  std::cout<<"DALKO false negtative rate:" <<DALKO_falseNegR<<std::endl;  
  std::cout<<"FALKO false negtative rate:" <<FALKO_falseNegR<<std::endl;
  
  return 0;
}
