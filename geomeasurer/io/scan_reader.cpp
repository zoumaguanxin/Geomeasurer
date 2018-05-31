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

#include "scan_reader.h"
#include <boost/graph/graph_concepts.hpp>
#include<assert.h>

namespace geomeasurer 
{
  namespace io 
  {
    
    std::istream& operator >> (std::istream &in, pose2dStamped& tempose)
    {      
    in>>tempose.head.name;

    in>>tempose.head.number>>tempose.x>>tempose.y>>tempose.theta;
    return in;
    }
    
    std::istream& operator >> (std::istream & file, sensor::rangeData & range_data)
    {      
       std::string lasername;
       file>>lasername;
        
       int num0; file>>num0;
       //file.seekg(2,std::ios::cur);
       file>>range_data.angle_min;
       double angle_range;
       file>>angle_range;
       range_data.angle_max=angle_range+range_data.angle_min;
       file>>range_data.angle_increment>>range_data.maxRange;
       double t1,t2;
       file>>t1>>t2;
       //  file.seekg(3,std::ios::cur);//这样不行，因为3指定的是字节，而每一位数的字节情况不一定。通用的处理是直接都处理成String
       int num; file>>num;
      //range_data.ranges.reserve(num);
      //file.seekg(4,std::ios::cur);
       for(int i=0;i<180;i++)
       {
	 double range;
	 file>>range;
	 range_data.ranges.emplace_back(range);
      }
      for(int i=0;i<13;i++)
      {
	double tem;
	file>>tem;
      }
            for(int i=0;i<3;i++)
	    {
		std::string endstring; 
	      file>>endstring;
	    }    
      return file;
    }
    
sensor::rangeData fromFile(const std::string& filedir)
{
     sensor::rangeData ret_range_data;
     std::ifstream file;
     file.open(filedir.c_str(), std::ios_base::in);
     if(file.good())
     {
      file>>ret_range_data.angle_min>>ret_range_data.angle_max>>ret_range_data.angle_increment;
      while(file.good())
      {  
	double range;
	 file>>range;
          ret_range_data.ranges.emplace_back(range);
      }
    }
    else
    {
      ::std::cout<<"can not open the file in specified directory"<<::std::endl;
      using namespace std;
     assert(file.good());
    }
    file.close();
    return ret_range_data;
}



RangesCorrespondingtoposes fromlogfile(const std::string& filedir)
{
     std::ifstream file;
     RangesCorrespondingtoposes rcps;
     file.open(filedir.c_str(), std::ios_base::in);
     if(file.good())
     {     
	while(file.good())
	{  
	  sensor::rangeData range_data;
	  pose2dStamped tempose;  
	  file>>tempose;
	  file>> range_data;
	  rangeWithpose rwp(tempose,range_data);
	  rcps.push_back(rwp);
	}
    }
    else
    {
      ::std::cout<<"can not open the file in the specified directory"<<::std::endl;
      using namespace std;
     assert(file.good());
    }
    file.close();
    return rcps;
}

    
  }// namespace io
}//namespace geomeasurer


