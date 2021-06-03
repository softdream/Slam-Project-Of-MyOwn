#include "slamProcessor.h"

namespace slam {

SlamProcessor::SlamProcessor()
{
	occupiedGridMap = new OccupiedMap();
	if( occupiedGridMap == nullptr ){
		std::cerr<<"Construct Occupied Grid Map Failed ..."<<std::endl;
		exit(-1);
	}
	std::cerr<<"Construct Occupied Grid Map Successfully ..."<<std::endl;

	scanMatch = new ScanMatchMethod();
	if( scanMatch == nullptr ){
		std::cout<<"Construct Scan Match Method Failed ..."<<std::endl;
		exit(-1);
	}	
	std::cerr<<"Construct Scan Match Method Successfully ..."<<std::endl;

	// ------------- initialize some parameters --------------//
	covarianceMatrix = Eigen::Matrix3f::Zero();
	lastScanMatchPose = Eigen::Vector3f::Zero();
	lastMapUpdatePose = Eigen::Vector3f::Zero();
}

SlamProcessor::~SlamProcessor()
{
	if( occupiedGridMap != nullptr ){
		delete occupiedGridMap;
	}
	
	if( scanMatch != nullptr ){
		delete scanMatch;
	}
}

SlamProcessor::SlamProcessor( int sizeX_, int sizeY_, float cellLength_ )
{
	occupiedGridMap = new OccupiedMap( sizeX_, sizeY_, cellLength_ );	
	if( occupiedGridMap == nullptr ){
                std::cerr<<"Construct Occupied Grid Map Failed ..."<<std::endl;
                exit(-1);
        }
        std::cerr<<"Construct Occupied Grid Map Successfully ..."<<std::endl;

	scanMatch = new ScanMatchMethod();
	if( scanMatch == nullptr ){
                std::cout<<"Construct Scan Match Method Failed ..."<<std::endl;
                exit(-1);
        }
        std::cerr<<"Construct Scan Match Method Successfully ..."<<std::endl;
	
	// ------------- initialize some parameters --------------//
        covarianceMatrix = Eigen::Matrix3f::Zero();
        lastScanMatchPose = Eigen::Vector3f::Zero();
        lastMapUpdatePose = Eigen::Vector3f::Zero();

}

void SlamProcessor::setUpdateLogOddsPoccValue( float Pocc )
{
	return occupiedGridMap->setLogOddsPoccValue( Pocc );
}

void SlamProcessor::setUpdateLogOddsPfreeValue( float Pfree )
{
	return occupiedGridMap->setLogOddsPfreeValue( Pfree );
}

void SlamProcessor::setMinDistanceDiffForMapUpdate( float minDist )
{
	minDistanceDiffForMapUpdate = minDist;
}

void SlamProcessor::setMinAngleDiffForMapUpdate( float minAngle )
{
	minAngleDiffForMapUpdate = minAngle;
}

void SlamProcessor::update( Eigen::Vector3f &robotPoseInWorld, 
			    ScanContainer &scanContainer,
			    bool mapWithoutMatching )
{
	// 1. Pose Scan Match
	Eigen::Vector3f newPoseEstimated;
	
	if( !mapWithoutMatching ){
		newPoseEstimated = scanMatch->scanToMap( *occupiedGridMap, robotPoseInWorld, scanContainer, covarianceMatrix, 100 );
	}
	else {
		newPoseEstimated = robotPoseInWorld;
	}
	
	lastScanMatchPose = newPoseEstimated;
	
	// 2. Map Update
	if( poseDiffLargerThan( lastMapUpdatePose, newPoseEstimated ) ){
		// update the map only when the pose change is greater than the threshol
		occupiedGridMap->updateByScan( scanContainer, newPoseEstimated );
		// occupiedGridMap->onMapUpdate();
		lastMapUpdatePose = newPoseEstimated;
	}
	
}

bool SlamProcessor::poseDiffLargerThan( Eigen::Vector3f &poseOld, Eigen::Vector3f &poseNew )
{
	if( ( ( poseNew.head<2>() - poseOld.head<2>() ).norm() ) > minDistanceDiffForMapUpdate ){
		return true;
	}

	float angleDiff = ( poseNew.z() - poseOld.z() );

	if( angleDiff > M_PI ){
		angleDiff -= M_PI * 2.0f;
	}
	else if( angleDiff < -M_PI ){
		angleDiff += M_PI * 2.0f;
	}

	if( ::abs( angleDiff ) > minAngleDiffForMapUpdate ){
		return true;
	}
	
	return false;
}

}
