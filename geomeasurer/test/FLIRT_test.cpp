#include <feature/Detector.h>
#include <feature/ShapeContext.h>
#include <feature/BetaGrid.h>
#include <feature/RangeDetector.h>
#include <feature/CurvatureDetector.h>
#include <feature/NormalBlobDetector.h>
#include <feature/NormalEdgeDetector.h>
#include <feature/RansacFeatureSetMatcher.h>
#include <feature/RansacMultiFeatureSetMatcher.h>
#include <sensorstream/CarmenLog.h>
#include <sensorstream/LogSensorStream.h>
#include <sensorstream/SensorStream.h>
#include <utils/SimpleMinMaxPeakFinder.h>
#include <utils/HistogramDistances.h>


#include <iostream>
#include <string>
#include <string.h>
#include <sstream>
#include <utility>
#include <sys/stat.h>
#include <sys/types.h>

#include "../io/scan_reader.h"
#include "../sensor/scan_data.h"
#include "../bridge/sensor_bridge.h"

#include "../viz/viz.h"


LogSensorStream m_sensorReference(NULL,NULL);

CurvatureDetector *m_detectorCurvature = NULL;
NormalBlobDetector *m_detectorNormalBlob = NULL;
NormalEdgeDetector *m_detectorNormalEdge = NULL;
RangeDetector *m_detectorRange = NULL;
Detector* m_detector = NULL;

BetaGridGenerator *m_betaGenerator = NULL;
ShapeContextGenerator *m_shapeGenerator = NULL;
DescriptorGenerator *m_descriptor = NULL;

RansacFeatureSetMatcher *m_ransac = NULL;

double angErrorTh = 0.2;
double linErrorTh = 0.5;

std::vector< std::vector<InterestPoint *> > m_pointsReference;
std::vector< OrientedPoint2D > m_posesReference;

unsigned int corresp[] = {0, 3, 5, 7, 9, 11, 13, 15};

double m_error[8] = {0.}, m_errorC[8] = {0.}, m_errorR[8] = {0.};
unsigned int m_match[8] = {0}, m_matchC[8] = {0}, m_matchR[8] = {0};
unsigned int m_valid[8] = {0};

struct timeval detectTime, describeTime, ransacTime;

unsigned int m_localSkip = 1;

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
    
   std::vector<int> groundth=io::groudtruthfromfile(filedir);
    int expectedNumkeypoints=std::accumulate(groundth.begin(),groundth.end(),0);
    int NumofdetectedKeypoints=0;    

   

     
    for(int iter=0;iter<groundth.size();iter++)
    {
   
   sensor::rangeData ranges=rcps[iter].second; 
   
    unsigned int scale = 5, dmst = 2, window = 3, detectorType = 0, descriptorType = 0, distanceType = 2, strategy = 0;
    double baseSigma = 0.2, sigmaStep = 1.4, minPeak = 0.34, minPeakDistance = 0.001, acceptanceSigma = 0.1, success = 0.95, inlier = 0.4, matchingThreshold = 0.4;
    bool useMaxRange = false; 
    
    SimpleMinMaxPeakFinder *m_peakMinMax = new SimpleMinMaxPeakFinder(minPeak, minPeakDistance);    
    
     std::string detector("");
     m_detectorCurvature = new CurvatureDetector(m_peakMinMax, scale, baseSigma, sigmaStep, dmst);
     m_detectorCurvature->setUseMaxRange(useMaxRange);
     m_detector = m_detectorCurvature;
     detector = "curvature";     

     
     LaserReading lasereading=sensor_bridge::fromRangeData(ranges);

     
     std::vector<InterestPoint*> keypoints;
     m_detector->detect(lasereading,keypoints);
     std::cout<<keypoints.size()<<std::endl;
     NumofdetectedKeypoints+=keypoints.size();
     
    PointCloud InterestPointPCD=sensor_bridge::fromInterstPoints(keypoints);
    
    viz::imshowPCDWithKeypoints("flirt",ranges,InterestPointPCD);
    }
     
     if(expectedNumkeypoints>NumofdetectedKeypoints)
     {
     double falseNegR=double(expectedNumkeypoints-NumofdetectedKeypoints)/double(expectedNumkeypoints);
       std::cout<<"FLIRT false negtative rate:" <<falseNegR<<std::endl;  
     }
     else{
         double falseNegR=double(NumofdetectedKeypoints-expectedNumkeypoints)/double(expectedNumkeypoints);
       std::cout<<"FLIRT false positive rate:" <<falseNegR<<std::endl;  
    }
//         std::string descriptor("");
// 	    HistogramDistance<double> *dist = NULL;
// 	    dist = new JensenShannonDistance<double>();
//         m_betaGenerator = new BetaGridGenerator(0.02, 0.5, 4, 12);
// 	    m_betaGenerator->setDistanceFunction(dist);
// 	    m_descriptor = m_betaGenerator;
// 	    descriptor = "beta";
// 	    
// 	    m_betaGenerator->describe();
    
  std::cout<<"hello world"<<std::endl;
  
}