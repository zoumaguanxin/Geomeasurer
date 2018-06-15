#include<falkolib/Matching/NNMatcher.h>
#include <boost/graph/graph_concepts.hpp>
#include <falkolib/Feature/FALKO.h>
#include<falkolib/Feature/FALKOExtractor.h>
#include<flirtlib/feature/RangeDetector.h>
#include"extractgeometryfeature.h"
#include "opencv/highgui.h"
#include "../viz/viz.h"



#include <falkolib/Feature/BSC.h>
#include <falkolib/Feature/FALKOExtractor.h>

#include <falkolib/Feature/BSCExtractor.h>

#include <falkolib/Matching/NNMatcher.h>
#include <falkolib/Matching/CCDAMatcher.h>

using  namespace  geomeasurer;
using namespace falkolib;
using namespace std;
 int main()
 {
        sensor::rangeData ranges1,ranges2;
        std::string filedir="../geomeasurer/data/dataflr4scan001";
        std::string filedir1="../geomeasurer/data/dataflr4scan003";

	ranges1=io::fromFile(filedir);
	ranges2=io::fromFile(filedir1);
        PointCloud scan2pcd1=sensor::fromRangeData(ranges1);
	PointCloud scan2pcd2=sensor::fromRangeData(ranges2);
        featurePointSet ret_keypoints1;
	 featurePointSet ret_keypoints2;
	falkolib::FALKOExtractor fe;
	fe.setMinScoreTh(0);
	fe.setMinExtractionRange(0.5);
	fe.setMaxExtractionRange(30);
	fe.enableSubbeam(false);
	fe.setNMSRadius(0.2);
	fe.setNeighB(0.07);
	fe.setBRatio(3);//2.5
	fe.setGridSectors(36);//16

	falkolib::LaserScan scan1(ranges1.angle_min,6*3.1419f/4, ranges1.ranges.size());
	scan1.fromRanges(ranges1.ranges);
	std::vector<falkolib::FALKO> keypoints1;
	fe.extract(scan1, keypoints1);
	for(falkolib::FALKO keypoint:keypoints1)
	{
	  ret_keypoints1.push_back(scan2pcd1.points[keypoint.index]);
	}	
	
	 
	 
	 
	falkolib::LaserScan scan2(ranges2.angle_min,6*3.1419f/4, ranges2.ranges.size());
	scan2.fromRanges(ranges2.ranges);
        std::vector<falkolib::FALKO> keypoints2;
        fe.extract(scan2, keypoints2);
	for(falkolib::FALKO keypoint:keypoints2)
	{
	  ret_keypoints2.push_back(scan2pcd2.points[keypoint.index]);
	}
	
	 cv::Mat img1,img2;
         img1=viz::imshowPCDWithKeypoints("faklo", ranges1,ret_keypoints1);
	img2=viz::imshowPCDWithKeypoints("faklo", ranges2,ret_keypoints2);

	cout << "num keypoints1 extracted: " << keypoints1.size() << endl;
	cout << "num keypoints2 extracted: " << keypoints2.size() << endl;

	
	BSCExtractor<FALKO> bsc(16, 8);
	vector<BSC> bscDesc1;
	vector<BSC> bscDesc2;


	bsc.compute(scan1, keypoints1, bscDesc1);
	bsc.compute(scan2, keypoints2, bscDesc2);




	NNMatcher<FALKO> matcher;
	matcher.setDistanceThreshold(0.1);
	std::vector<std::pair<int, int> > assoNN;
	std::cout << "num matching NN: " << matcher.match(keypoints1, keypoints2, assoNN) << endl;
	for (auto& match : assoNN) {
		if (match.second >= 0) {
			int i1 = match.first;
			int i2 = match.second;
			std::cout << "i1: " << i1 << "\ti2: " << i2 << "\t keypoints distance: " << (keypoints1[i1].distance(keypoints2[i2])) << endl;
		}
	}
	/*
     std::vector<std::pair<point3d,point3d> > kppairs; 
    for(auto pair:assoNN)
    {
      int index1, index2;
      double score;     
      point3d temp1=scan2pcd1.points[pair.first];
      point3d temp2=scan2pcd2.points[pair.second];
      kppairs.push_back(std::make_pair(temp1,temp2));
      std::cout<<"index1:"<<index1<<"      "<<"index2:"<<index2<<"    "<<"score:"<<score<<std::endl;
    }    
    viz::imshowMatch(img1,img2,kppairs);*/

	cout << endl;
	CCDAMatcher<FALKO> cc;
	cc.setDistMin(0.05);
	cc.setDistTol(0.1);
	std::vector<std::pair<int, int> > assoCC;
	std::cout << "num matching CC: " << cc.match(keypoints1, keypoints2, assoCC) << endl;
	for (auto& match : assoCC) {
		if (match.second >= 0) {
			int i1 = match.first;
			int i2 = match.second;
			std::cout << "i1: " << i1 << "\ti2: " << i2 << "\t keypoints distance: " << (keypoints1[i1].distance(keypoints2[i2])) << endl;
		}
	}
    std::vector<std::pair<point3d,point3d> > kppairs1; 
    for(auto pair:assoCC)
    {
      int index1, index2;
      double score;     
      point3d temp1=scan2pcd1.points[keypoints1[pair.first].index];
      point3d temp2=scan2pcd2.points[keypoints2[pair.first].index];
      kppairs1.push_back(std::make_pair(temp1,temp2));    
    }    
    viz::imshowMatch(img1,img2,kppairs1);
    
    
}