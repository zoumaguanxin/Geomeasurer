#include <boost/graph/graph_concepts.hpp>
#include<iostream>

#include "repeatabilitytest.h"
#include "../io/scan_reader.h"

using namespace geomeasurer;

int main()
{
    // std::string filename="../geomeasurer/data/fr079.log";   
  // std::string filename="../geomeasurer/data/mit-cscail.log";   
   std::string filename="../geomeasurer/data/intel-lab.log";   
   // std::string filedir="../geomeasurer/data/groundtruth_fr079";
   std::string filedir="../geomeasurer/data/groundtruth_intel_lab";
   // std::string filedir="../geomeasurer/data/groundtruth_mit-csail";
   sensor::RangesCorrespondingtoposes rcps;     
 
   
   rcps=io::fromlogfile(filename);  
   
   
   
     int interval=1;
     std::vector<double> DALKO_repeatability,FALKO_repeatability,FLIRT_repeatability;
    for(int iter=20;iter<21;iter++)
    {
   sensor::rangeWithpose ranges_ref,ranges_src;
   ranges_ref=rcps[iter]; ranges_src=rcps[iter+interval];
   
   sensor::rangeData ranges=rcps[iter].second;    

   
   
   //DALKO
   RepeatabilityTestDALKO retestdalko(4,0.045);
   retestdalko.setKpAssociationDistThres(0.1);   
   
   retestdalko.setTwoRangesWithPose(ranges_ref,ranges_src);   
   if(retestdalko.detect())
   {
      double dalko_detector_re= retestdalko.getRepeatabilityDetecor();
      DALKO_repeatability.push_back(dalko_detector_re);
   }
   
   //FALKO
   RepeatabilityTestFALKO retestfalko(4,0.045);
   retestfalko.setKpAssociationDistThres(0.1);
   retestfalko.setTwoRangesWithPose(ranges_ref,ranges_src);
   if(retestfalko.detect())
   {
        double falko_detector_re= retestfalko.getRepeatabilityDetecor();
      FALKO_repeatability.push_back(falko_detector_re);
  }
   
    RepeatabilityTestFLIRT retestflirt(4,0.045);
   retestflirt.setKpAssociationDistThres(0.1);
   retestflirt.setTwoRangesWithPose(ranges_ref,ranges_src);
   if(retestflirt.detect())
   {
        double flirt_detector_re= retestflirt.getRepeatabilityDetecor();
      FLIRT_repeatability.push_back(flirt_detector_re);
  }
   
   
  }
  
  
  
  for(auto re_tem:DALKO_repeatability)
  {
    std::cout<<"DALKO detector  repeatability test reuslt:"<<re_tem<<std::endl;
  }
     for(auto re_tem:FALKO_repeatability)
  {
    std::cout<<"FALKO detector  repeatability test reuslt:"<<re_tem<<std::endl;
  }
    
       for(auto re_tem:FLIRT_repeatability)
  {
    std::cout<<"FLIRT detector  repeatability test reuslt:"<<re_tem<<std::endl;
  }
}