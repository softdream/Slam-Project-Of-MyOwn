#ifndef __SLAM_PROCESSOR_H_
#define __SLAM_PROCESSOR_H_

#include "scanMatch.h"
#include "dataType.h"

#include <opencv2/opencv.hpp>

namespace slam {

class SlamProcessor
{
public:
	SlamProcessor();
	SlamProcessor( int sizeX_, int sizeY_, float cellLength_ );
	
	~SlamProcessor();

	// ------------- Set Parameters ------------- //
	void setUpdateLogOddsPoccValue( float Pocc );
	void setUpdateLogOddsPfreeValue( float Pfree );
	void setMinDistanceDiffForMapUpdate( float minDist );
	void setMinAngleDiffForMapUpdate( float minAngle );

	// ------------- Update --------------//
	void update( Eigen::Vector3f &robotPoseInWorld, 
		     ScanContainer &scanContainer,
		     bool mapWithoutMatching = false );

	// ---------- Get Map Information ---------//
	MapInfo getMapInfo() const;	

	// ---------- Process the first laser scan ------------//
	void processTheFirstScan( Eigen::Vector3f &robotPoseInWorld,
				  ScanContainer &scanContainer );
	
	void laserData2Container( const slam::sensor::LaserScan &scan, slam::ScanContainer &container );

//	Eigen::Vector3f getLastScanMatchPose() const;
	const Eigen::Vector3f getLastScanMatchPose() const;

//	Eigen::Vector3f getLastMapUpdatePose() const;
	const Eigen::Vector3f getLastMapUpdatePose() const;

	void displayMap() const;

private:
	bool poseDiffLargerThan( Eigen::Vector3f &poseOld, Eigen::Vector3f &poseNew );

	//void laserData2Container( const slam::sensor::LaserScan &scan, slam::ScanContainer &container );

private:

	// - Grid Map Object & Scan Match Object -//
	OccupiedMap *occupiedGridMap;
	ScanMatchMethod *scanMatch;

	// ------------ Parameters -------------- //
	float minDistanceDiffForMapUpdate;
	float minAngleDiffForMapUpdate;
	

	Eigen::Matrix3f covarianceMatrix;	
	Eigen::Vector3f lastScanMatchPose;
	Eigen::Vector3f lastMapUpdatePose;
	
	// ------------ opencv map -------------- //
	cv::Mat image;
}; 

}

#endif
