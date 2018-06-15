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

#ifndef PREDEAL_H
#define PREDEAL_H
#include<Eigen/Dense>
#include<pcl/common/common.h>
#include <cstring>
#include<vector>
#include<Eigen/Geometry>
using namespace std;
namespace predeal
{
  
  /**
   * @brief 读取pose文件，pose文件按照3个自由度来存储，前两个是位移，第三个是旋转
   * @param dir 文件所在位置
   * @param v_pose 用一个向量容器来存放
   */
  void readposefile(const string& dir, vector<Eigen::Vector3f> &v_pose);
  
  
  /**
   * @brief 读取点云文件，直接存放为XYZ格式
   * @param dir 文件所在目录
   * @param pcd 存放点云
   */
  void readpcdfile(const string&dir, pcl::PointCloud<pcl::PointXYZ> &pcd);
  
  
  
 /**
  *@brief 将存放为3维向量的pose转换为旋转平移
  * @param[in] pose 3维的向量
  * @param[out] R 旋转
  * @param[out] t 平移
  */ 
  void vector3ftoRationTtrans(const Eigen::Vector3f &pose, Eigen::Matrix3f &R, Eigen::Vector3f &t);
  
  /**
   * @brief 通过两个位姿计算相对位姿, 就算出src_pose相对于tgt_pose的位姿，用这个位姿，可以把src_frame下的点通过RX+T转换到tgt_frame下
   * @param src_pose  
   * @param tgt_pose
   * @note 当只是返回一个值的时候不要用地址符号，由于函数块声明的变量的生命周期之是在函数运行的阶段，返回引用说明将来需要使用它，
   * 但由于函数结束，该引用指向的对象被释放，很容易导致程序出现问题但是不会报错
   * 
   */
  Eigen::Vector3f computeConstraint( const Eigen::Vector3f tgt_pose,const Eigen::Vector3f &src_pose);
  
  
  
  
  
  
};

#endif // PREDEAL_H
