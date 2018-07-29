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

#include "transform.h"

namespace geomeasurer 
{
     void transformpcd(const pcl::PointCloud<pcl::PointXYZ>::Ptr src_pcp, pcl::PointCloud<pcl::PointXYZ>::Ptr dst_pcp,const Eigen::Vector3f &tranlsation, const Eigen::Matrix3f &Rotation)
   { 
     for(int j=0;j<src_pcp->points.size();++j)
	{
	  Eigen::Vector3f temV;
	  temV<<src_pcp->points[j].x,src_pcp->points[j].y,src_pcp->points[j].z;
	  temV=Rotation*temV;
	  pcl::PointXYZ temPoint;
	  temPoint.x=temV(0)+tranlsation(0);
	  temPoint.y=temV(1)+tranlsation(1);
	  temPoint.z=temV(2)+tranlsation(2);
	  dst_pcp->points.push_back(temPoint);
	}
  }
  
}