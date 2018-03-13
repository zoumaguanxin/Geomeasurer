#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include "../sensor/scan_data.h"
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2//core/core.hpp>

namespace geomeasurer{

  namespace viz{
    
  
boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> MutiVis (std::vector<pcl::PointCloud<pcl::PointXYZ>::ConstPtr > cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "registration cloud");
  int step=(255+255+255)/cloud.size();
  int R,G,B;
  for(int i=0;i<cloud.size();i++)
  {
    std::string cloudName;
   char cloudname[50];
   sprintf(cloudname,"cloud%.3d",i);
    if(step*(i+1)<=255)
    {
      R=(i+1)*step;
      G=0;
       B=0;
    }
   else if(step*(i+1)>=255&&step*(i+1)<=510)
    {
     R=255;
     G=(i+1)*step-255;
     B=0;
    }
    else{
       R=255;
      G=255;
      B=(i+1)*step-510;
    }
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(cloud[i], R, G, B);
    viewer->addPointCloud<pcl::PointXYZ>(cloud[i],color,cloudName);
  }
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> twoVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pc1, pcl::PointCloud<pcl::PointXYZ>::ConstPtr pc2)
{
 boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("point cloud"));
  viewer->setBackgroundColor(0,0,0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red_color(pc1, 255, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green_color(pc2, 0,255,0);
  //重载函数
 viewer->addPointCloud<pcl::PointXYZ>(pc1,red_color,"pcd1");
 
 //首先设置渲染特性，PCL_VISUALIZER_POINT_SIZE是一种. 
 viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1,"pcd1");
 //注意后面这个必须在命名时与上面的不同，不然与上面的点云颜色相同
 viewer->addPointCloud<pcl::PointXYZ >(pc2,green_color,"pcd2");
 viewer->addCoordinateSystem(1.0);
 viewer->initCameraParameters();
return viewer;  
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> normalVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pc, pcl::PointCloud<pcl::Normal>::ConstPtr Normals)
{
 boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("point with Normal"));
  viewer->setBackgroundColor(0,0,0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(pc, 0, 255, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::Normal> n_color(Normals, 0,0,255);
  //重载函数
 viewer->addPointCloud<pcl::PointXYZ>(pc,single_color,"sample cloud");
 
 //首先设置渲染特性，PCL_VISUALIZER_POINT_SIZE是一种. 
 viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1,"sample cloud");
 //注意后面这个必须在命名时与上面的不同，不然与上面的点云颜色相同
 viewer->addPointCloudNormals<pcl::PointXYZ,pcl::Normal >(pc,Normals,10,5,"normals");
 viewer->addCoordinateSystem(1.0);
 viewer->initCameraParameters();
return viewer;  
}

void imshowPCDFromRanges(const sensor::rangeData &ranges)
{
  
}

void imshowPCDWithKeypoints(const std::string & name, const sensor::rangeData& ranges, const PointCloud& features )
{
 auto max_range_Iter=std::max_element(ranges.ranges.begin(),ranges.ranges.end());
  int width=300+*(max_range_Iter)/0.02f;//注意改为200会报错
  //对应的还有CV_8UC1,CV_8UC2,CV_BUC3
  cv::Mat img(cv::Size(width,width),CV_8UC3,cv::Scalar(255,255,255));  
   PointCloud candidate_pcd=sensor::fromRangeData(ranges);
  for(auto point:candidate_pcd.points)
  {
    int u=img.cols/2+floor(point.x/0.02f);
    int v=img.rows/2+floor(point.y/0.02f);
    img.at<cv::Vec3b>(v,u)=cv::Vec3b(255,0,0);
    //img.at<uchar>(v,u)=0;
  }  
  for(point3d keypoint:features.points)
  {
    cv::Point2i point;
    point.x=img.cols/2+floor(keypoint.x/0.02f);
    point.y=img.cols/2+floor(keypoint.y/0.02f);
    cv::circle(img, point,5,cv::Scalar(0,0,255));
  }
  //cv::Mat img_part=img(cv::Range(500,1500),cv::Range(500,1500));
  cv::Mat dst_img;
  cv::resize(img,dst_img,cv::Size(),2,2,cv::INTER_CUBIC);  
  cv::namedWindow(name,cv::WINDOW_AUTOSIZE);
  cv::imshow(name,img);  
  cv::waitKey();  
 }
} //viz

}//map3d