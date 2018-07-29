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

#include "scan_data.h"
namespace geomeasurer {
  namespace sensor {    

    
PointCloud fromRangeData(const rangeData& ranges_data)
{
  PointCloud ret;
  double angle;
  int count=0;
  for(auto range : ranges_data.ranges)
  {
    angle=ranges_data.angle_min+count*ranges_data.angle_increment;
    point3d temPoint;
    temPoint.x=range*cos(angle);
    temPoint.y=range*sin(angle);
    ret.push_back(temPoint);
   count++;    
  }  
  return ret;
}

rangeData fromPointCloud(const PointCloud& pcd)
{
  rangeData scan;
  for(auto pk:pcd.points)
  {
       double range=sqrt(pow(pk.x,2)+pow(pk.y,2)+pow(pk.z,2));
       scan.ranges.push_back(range);
  }
  return scan;
}


int rangeData::getIndexbyPoint(const point3d& point)
{
   pcl::KdTreeFLANN<point3d> kdtree;
   assert(!ranges.empty());
   PointCloud tempcd=getpcdfromscan();
   kdtree.setInputCloud(tempcd.makeShared());
   int k=1;std::vector<int> indexes;std::vector<float> squaredDistances;
   kdtree.nearestKSearch(point,1,indexes,squaredDistances);
   return indexes[0];
}

PointCloud rangeData::getpcdfromscan()
{
  PointCloud ret;
  double angle;
  int count=0;
  for(auto range : ranges)
  {
    angle=angle_min+count*angle_increment;
    point3d temPoint;
    temPoint.x=range*cos(angle);
    temPoint.y=range*sin(angle);
    ret.push_back(temPoint);
   count++;    
  }  
  return ret;
}



}//namespace sensor  
}//namespace geomeasurer


