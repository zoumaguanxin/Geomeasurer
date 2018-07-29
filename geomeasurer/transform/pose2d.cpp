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

#include "pose2d.h"

#include <iostream>

pose2d::pose2d(const double& x_, const double& y_, const double& theta_)
{
   x=x_;
   y=y_;
   theta=theta_;
}


double pose2d::getx()
{
  return x;
}

double pose2d::gety()
{
  return y;
}

double pose2d::gettheta()
{
  return theta;
}

Eigen::Matrix3f pose2d::getRoationMatrix() const
{
  //不使用中间变量R时, 直接return时报错
   Eigen::Matrix3f R;
    R=Eigen::AngleAxisf(float(theta),Eigen::Vector3f::UnitZ());
    return R;
}

Eigen::Vector3f pose2d::getVec3f() const
{  
return Eigen::Vector3f(x,y,0.d);
}

pose2d getPoseEdge(const pose2d& p_ref, const pose2d& p_src)
{
  

   Eigen::Matrix3f R_ref=p_ref.getRoationMatrix(); 
   Eigen::Matrix3f R_src=p_src.getRoationMatrix();   
   
   Eigen::Matrix3f R_ref_src=R_ref.transpose()*R_src;
   
   Eigen::Vector3f t_ref=p_ref.getVec3f();
   Eigen::Vector3f t_src=p_src.getVec3f();
    
   Eigen::Vector3f t_src_ref=R_ref.transpose()*(t_src-t_ref);
   
   Eigen::Quaternion<float> q(R_ref_src);
   
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
	std::cout<<"四元数解析错误，该旋转轴不为z轴"<<std::endl;
	  exit(0);
	}

    v(1)=t_src_ref(1);
    v(0)=t_src_ref(0);
    pose2d pose(v(0),v(1),v(2));
    return pose;   
   
}




