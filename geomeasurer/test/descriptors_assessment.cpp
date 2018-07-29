#include <boost/graph/graph_concepts.hpp>
#include<iostream>

#include "repeatabilitytest.h"
#include "../io/scan_reader.h"

using namespace geomeasurer;

int main()
{
  
  //考虑随机抽100组
    //std::string filename="../geomeasurer/data/fr079.log";   
  //std::string filename="../geomeasurer/data/mit-cscail.log";   //2102个数据
    std::string filename="../geomeasurer/data/intel-lab.log";   //5344个数据
   // std::string filedir="../geomeasurer/data/groundtruth_fr079";
   std::string filedir="../geomeasurer/data/groundtruth_intel_lab";
   // std::string filedir="../geomeasurer/data/groundtruth_mit-csail";
   sensor::RangesCorrespondingtoposes rcps;     
 
   
   rcps=io::fromlogfile(filename);  
   
   
   
     int interval=1;
     std::vector<double> DALKO_repeatability,FALKO_repeatability,FLIRT_repeatability, DALKO_desassess;
    for(int iter=1;iter<100;iter++)
    {
   sensor::rangeWithpose ranges_ref,ranges_src;
   ranges_ref=rcps[iter]; ranges_src=rcps[iter+interval];
   
   sensor::rangeData ranges=rcps[iter].second;    

   
   
   //DALKO
   RepeatabilityTestDALKO retestdalko(4,0.045);
   retestdalko.setKpAssociationDistThres(0.1);   
   
   retestdalko.setTwoRangesWithPose(ranges_ref,ranges_src);   
     // assert(retestdalko.detect());
   if(retestdalko.detect())
   {
      
      double dalko_detector_re= retestdalko.getRepeatabilityDetecor();
      retestdalko.match();
     double x=retestdalko.getRepeatabilityDescriptos();
     DALKO_desassess.push_back(x);
      DALKO_repeatability.push_back(dalko_detector_re);
   }
   
//    //FALKO
//    RepeatabilityTestFALKO retestfalko(4,0.045);
//    retestfalko.setKpAssociationDistThres(0.1);
//    retestfalko.setTwoRangesWithPose(ranges_ref,ranges_src);
//    if(retestfalko.detect())
//    {
//         double falko_detector_re= retestfalko.getRepeatabilityDetecor();
//       FALKO_repeatability.push_back(falko_detector_re);
//   }
//    
//     RepeatabilityTestFLIRT retestflirt(4,0.045);
//    retestflirt.setKpAssociationDistThres(0.1);
//    retestflirt.setTwoRangesWithPose(ranges_ref,ranges_src);
//    if(retestflirt.detect())
//    {
//         double flirt_detector_re= retestflirt.getRepeatabilityDetecor();
//       FLIRT_repeatability.push_back(flirt_detector_re);
//   }
   
  }
  
  
  assert(!DALKO_repeatability.empty());
  double e_DALKO=0,e_FALKO=0, e_FLIRT=0;
  for(auto re_tem:DALKO_repeatability)
  {
  //  std::cout<<"DALKO detector  repeatability test reuslt:"<<re_tem<<std::endl;
    e_DALKO+=re_tem;
  }
  e_DALKO/=DALKO_repeatability.size();
  std::cout<<"expect DALKO detector  repeatability "<<e_DALKO<<std::endl;
  double e_re_des=0;
  for(auto re_des:DALKO_desassess)
  {
    e_re_des+=re_des;    
  }
  
  e_re_des/=DALKO_desassess.size();
  std::cout<<"expect DALKO des  repeatability "<<e_re_des<<std::endl;
  
//      for(auto re_tem:FALKO_repeatability)
//   {
//     e_FALKO+=re_tem;
//     std::cout<<"FALKO detector  repeatability test reuslt:"<<re_tem<<std::endl;
//   }
//    e_FALKO/=FALKO_repeatability.size(); 
//        for(auto re_tem:FLIRT_repeatability)
//   {
//     e_FLIRT+=re_tem;
//     //std::cout<<"FLIRT detector  repeatability test reuslt:"<<re_tem<<std::endl;
//   }
//   e_FLIRT/=FLIRT_repeatability.size();
//   
//   std::cout<<"expect DALKO detector  repeatability "<<e_DALKO<<std::endl;
//    std::cout<<"expect FALKO detector  repeatability "<<e_FALKO<<std::endl;
//     std::cout<<"expect FLIRT detector  repeatability "<<e_FLIRT<<std::endl;
  
}