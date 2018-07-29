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

#include "predeal.h"
#include<fstream>
#include <boost/concept_check.hpp>
using namespace std;
void predeal::readpcdfile(const string& dir, pcl::PointCloud< pcl::PointXYZ >& pcd)
{
  ifstream file;
  file.open(dir.c_str(), ios_base::in);
  if(file.good())
  {
      while(file.good())
      {
	pcl::PointXYZ tempoint;
	file>>tempoint.x>>tempoint.y>>tempoint.z;
	pcd.push_back(tempoint);
      }
  }
  else
  {
    cout<<"can't open the file in the specified dir"<<endl;
    assert(file.good());
  }
}

void predeal::readposefile(const string& dir, vector<Eigen::Vector3f> &v_pose)
{

  ifstream file;  
  file.open(dir.c_str(),ios_base::in);  
  int count_f=0;
  if(file.good())
  {
      while(file.good())
      {
       Eigen::Vector3f tempose;
       if(count_f==0)
       {
	 file>>tempose(0);
	 count_f++;
	 if(file.eof())
	 {
	   cout<<"文件为空"<<endl;
	   file.close();
	  }
	  else
	  {
	    file>>tempose(1)>>tempose(2);
	  }
       }
       else
       {
	   file>>tempose(0)>>tempose(1)>>tempose(2);
       }
       v_pose.push_back(tempose);
      }
  }
  else
  {
    cout<<"can't open the file in the specified dirctory"<<endl;
    assert(file.good());
  }
  file.close();
}

void predeal::vector3ftoRationTtrans(const Eigen::Vector3f& pose, Eigen::Matrix3f& R, Eigen::Vector3f& t)
{
   Eigen::Quaternion<float> q_w_target(cos(pose(2)/2),0,0,sin(pose(2)/2));
   t(0)=pose(0);
   t(1)=pose(1);
   t(2)=0.0f;
  R=q_w_target.toRotationMatrix();
}


Eigen::Vector3f predeal::computeConstraint( const Eigen::Vector3f tgt_pose,const Eigen::Vector3f& src_pose)
{
	Eigen::Quaternion<float> q_w_target(cos(tgt_pose(2)/2),0,0,sin(tgt_pose(2)/2));
	Eigen::Quaternion<float> q_w_source(cos(src_pose(2)/2),0,0,sin(src_pose(2)/2));
	Eigen::Vector3f t_w_target, t_w_source;
	t_w_target<<tgt_pose(0), tgt_pose(1),0.0f;
	t_w_source<<src_pose(0), src_pose(1),0.0f;
	Eigen::Matrix3f R_initial=q_w_target.toRotationMatrix().transpose()*q_w_source.toRotationMatrix();
	Eigen::Vector3f t_initial=q_w_target.toRotationMatrix().transpose()*(t_w_source-t_w_target);
	cout<<t_initial<<endl;
	Eigen::Quaternion<float> q(R_initial);
        Eigen::Vector3f v;
	//以下的处理非常重要，只适用于只存在偏航角的情况
	float a1=acos(q.w());
	float a2=-a1;
	if(abs(sin(a1)-q.z())<0.001f)
	{
	  v(2)=2*a1;

	}
	else if(abs(sin(a2)-q.z())<0.001f)
	{
	  v(2)=2*a2;
	  //cout<<v(2)<<endl;
	}
	else{
	  cout<<"四元数解析错误，该旋转轴不为z轴"<<endl;
	  exit(0);
	}

    v(1)=t_initial(1);
    v(0)=t_initial(0);
    return v;
}



