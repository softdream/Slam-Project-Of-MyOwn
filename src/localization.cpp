#include "localization.h"

namespace slam{

Localization::Localization()
{
	// 1. get a known map from the storaged file
	// TODO ....
}

Localization::~Localization()
{
	if( occupiedGridMap != nullptr ){
                delete occupiedGridMap;
        }

        if( scanMatch != nullptr ){
                delete scanMatch;
        }
	
	std::cerr<<"delete the occupied grid map memory ... "<<std::endl;
}

void Localization::setInitialPose(const Eigen::Vector3f &pose)
{
	lastScanMatchPose = pose;

	isPoseInitialized = true;
}

bool Localization::update( const ScanContainer &scanContainer, bool mapWithoutMatching )
{
	if( !isPoseInitialized ){
		return false;
	}	

	// 1. Pose Scan Match
	Eigen::Vector3f newPoseEstimated;// estimated pose in world coordinate

        if( !mapWithoutMatching ){ 
		// 2. scan Match
                newPoseEstimated = scanMatch->scanToMap( *occupiedGridMap,  // map
							 lastScanMatchPose, // robot pose in world coordination
							 scanContainer,     // scan data
							 covarianceMatrix,  // covariance matrix
							 100 );		    // iteration 
        }
        else {
                newPoseEstimated = lastScanMatchPose;
        }

	// 3. update the pose in world coordination
        lastScanMatchPose = newPoseEstimated;
	
	return true;
}

const Eigen::Vector3f Localization::getLastScanMatchPose() const
{
        return lastScanMatchPose;
}

const Eigen::Matrix3f Localization::getCovarianceMatrix() const 
{
	return covarianceMatrix;
}



}
