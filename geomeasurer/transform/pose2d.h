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

#ifndef POSE2D_H
#define POSE2D_H
#include <boost/concept_check.hpp>
#include <boost/graph/graph_concepts.hpp>
#include<eigen3/Eigen/Dense>

class pose;
struct header{
  double time;
  double frame;
  std::string name;
  int number;
};

class pose2d
{
  public:
  double x;
  double y;
  double theta;
  double getx();
  double gety();
  double gettheta();
  pose2d(){}
  pose2d(const double &x_, const double &y_, const double &theta_);
  
  Eigen::Matrix3f getRoationMatrix() const;
  Eigen::Vector3f getVec3f() const; 
  ~pose2d(){}
};

class pose2dStamped:public pose2d
{
  public:
  header head;
  pose2dStamped():pose2d(){}
  pose2dStamped(const double& x_, const double& y_, const double& theta_):pose2d(x_,y_,theta_)
    {}

~pose2dStamped(){}  
};

pose2d getPoseEdge(const pose2d& p_ref, const pose2d& p_src);

#endif // POSE2D_H
