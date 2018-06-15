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

#include "viz.h"



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

cv::Mat imshowPCDWithKeypoints(const std::string & name, const sensor::rangeData& ranges, const PointCloud& features )
{
 auto max_range_Iter=std::max_element(ranges.ranges.begin(),ranges.ranges.end());
 double max_range=15;
 
 // int width=20+2*(*(max_range_Iter))/0.02f;
 int width=20+2*(max_range)/0.02f;
  //对应的还有CV_8UC1,CV_8UC2,CV_BUC3，opencv中对应为GBR
  cv::Mat img(cv::Size(width,width),CV_8UC3,cv::Scalar(0,0,0));  
   PointCloud candidate_pcd=sensor::fromRangeData(ranges);
  for(auto point:candidate_pcd.points)
  {
    if(math::rangefromcartesian(point)<max_range)
    {
      int u=img.cols/2+floor(point.x/0.02f);
      int v=img.rows/2+floor(point.y/0.02f);
      img.at<cv::Vec3b>(v,u)=cv::Vec3b(255,255,255);
    }
    //img.at<uchar>(v,u)=0;
  }  
  for(point3d keypoint:features.points)
  {
    if(math::rangefromcartesian(keypoint)<max_range){
    cv::Point2i point;
    point.x=img.cols/2+floor(keypoint.x/0.02f);
    point.y=img.rows/2+floor(keypoint.y/0.02f);
    cv::circle(img, point,5,cv::Scalar(0,0,255));
    }
  }
  //cv::Mat img_part=img(cv::Range(500,1500),cv::Range(500,1500));
  cv::Mat dst_img;
  cv::resize(img,dst_img,cv::Size(),2,2,cv::INTER_CUBIC);  
  cv::namedWindow(name,cv::WINDOW_AUTOSIZE);
  cv::imshow(name,img);  
  cv::waitKey();  
  return img;
 }
 
 
 
 void imshowMatch(const cv::Mat &img1,  const cv::Mat &img2, std::vector<std::pair<point3d,point3d> > pairs)
{
  //cv::Mat trim_img1=img1.clone();
  int cols=img1.cols+img2.cols;  
   int rows=std::max(img1.rows,img2.rows);
   cv::Mat composedImg(cv::Size(cols,rows),CV_8UC3,cv::Scalar(0,0,0));
   
   cv::Mat img_left, img_right;
   img_left=composedImg(cv::Rect(0,0,img1.rows,img1.cols));
   img_right=composedImg(cv::Rect(img1.cols,0, img2.cols, img2.rows));
  
   img1.copyTo(img_left);
   img2.copyTo(img_right);
   
   for(std::pair<point3d,point3d> pair:pairs)
   {
     cv::Point2i p1,p2;
     p1.x=img1.cols/2+floor(pair.first.x/0.02); p2.x=img1.cols+img2.cols/2+floor(pair.second.x/0.02);
     p1.y=img1.cols/2+floor(pair.first.y/0.02); p2.y=img2.cols/2+floor(pair.second.y/0.02);
     cv::line(composedImg,p1,p2,cv::Scalar(0,255,0));     
  }   

  
   cv::Mat dst_img;
   cv::resize(composedImg,dst_img,cv::Size(),0.5,0.5,cv::INTER_CUBIC);
   cv::namedWindow("match",cv::WINDOW_AUTOSIZE);
   cv::imshow("match",dst_img);
   cv::waitKey();    
}
 
} //viz




}//map3d