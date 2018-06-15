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

#ifndef VIZ_H
#define VIZ_H

#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include "../sensor/scan_data.h"
#include "../common/math_supplement.h"
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2//core/core.hpp>

namespace geomeasurer{

  namespace viz{
    
  
boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);


boost::shared_ptr<pcl::visualization::PCLVisualizer> MutiVis (std::vector<pcl::PointCloud<pcl::PointXYZ>::ConstPtr > cloud);


boost::shared_ptr<pcl::visualization::PCLVisualizer> twoVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pc1, pcl::PointCloud<pcl::PointXYZ>::ConstPtr pc2);


boost::shared_ptr<pcl::visualization::PCLVisualizer> normalVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pc, pcl::PointCloud<pcl::Normal>::ConstPtr Normals);


void imshowPCDFromRanges(const sensor::rangeData &ranges);


cv::Mat imshowPCDWithKeypoints(const std::string & name, const sensor::rangeData& ranges, const PointCloud& features );

 
 
 void imshowMatch(const cv::Mat &img1,  const cv::Mat &img2, std::vector<std::pair<point3d,point3d> > pairs);

 
} //viz



}//map3d

#endif // VIZ_H
