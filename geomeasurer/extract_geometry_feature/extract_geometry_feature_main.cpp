#include "extractgeometryfeature.h"

#include "../viz/viz.hpp"
#include <cstring>
#include <opencv/highgui.h>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2//core/core.hpp>
#include <boost/graph/graph_concepts.hpp>
using namespace geomeasurer;
int main()
{ 

  cv::Mat img(1000,1000,CV_8UC1, cv::Scalar(255)); 

  sensor::rangeData ranges;
  std::string filedir="../geomeasurer/data/scan003";

  ranges=io::fromFile(filedir);
  extractGeometryFeature gfs(ranges);

    gfs.setRegionGrowRadius(0.2);
    featurePointSet featurePoints=gfs.extractgfs("IFAKLO");
    featurePointSet featurePoints_falko=gfs.extractgfs("FAKLO"); 
// featurePointSet featurePoints_rangeResponsed=gfs.extractFLIRT(); 


 viz::imshowPCDWithKeypoints("ifaklo", ranges,featurePoints);
 viz::imshowPCDWithKeypoints("faklo",ranges,featurePoints_falko);

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