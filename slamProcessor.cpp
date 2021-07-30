#include "slamProcessor.h"

namespace slam {

SlamProcessor::SlamProcessor(): minDistanceDiffForMapUpdate( 0.4 ),
				minAngleDiffForMapUpdate( 0.9 ) // default parameter
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

SlamProcessor::SlamProcessor( int sizeX_, int sizeY_, float cellLength_ ): minDistanceDiffForMapUpdate( 0.4 ),
                                					   minAngleDiffForMapUpdate( 0.9 ) // default parameter
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
	Eigen::Vector3f newPoseEstimated;// estimated pose in world coordinate
	
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

MapInfo SlamProcessor::getMapInfo() const
{
	return occupiedGridMap->getMapInfo();
}

void SlamProcessor::processTheFirstScan( Eigen::Vector3f &robotPoseInWorld, 
                                  	 ScanContainer &scanContainer )
{
	// if it is the first laser scan, just update to initialize the map 
	occupiedGridMap->updateByScan( scanContainer, robotPoseInWorld );
}

void SlamProcessor::laserData2Container( const slam::sensor::LaserScan &scan, slam::ScanContainer &container )
{
	std::cout<<"------------------ Laser Data To Container -----------------"<<std::endl;
	container.clear();

	float theta = -std::fabs(-3.12144f - 3.14159f) / 360;
	//float theta = std::fabs( scan.angle_min );
	//std::cout<<"theta = "<<theta<<std::endl;
	
	for( int i = 0; i < scan.size(); ++ i ){
		float dist = scan.ranges[ i ];
	//	std::cout<<"distance = "<<dist<<std::endl;		

		//if( dist >= scan.range_min && dist <= scan.range_max ){
		if( dist >= 0.1f && dist <= 12.0f ){
			Eigen::Vector2f point( ::cos( theta ) * dist, ::sin( theta ) * dist );
	//		std::cout<<"laser point: ( "<<point[0]<<", "<<point[1]<<" )"<<std::endl;
			
			container.addData( point );
		}
		
	//	theta += std::fabs( scan.angle_increment );
		theta += std::fabs( 0.0174533f );
	}
	
	 std::cout<<"Scan Container Size: "<<container.getSize()<<std::endl;
}



/*Eigen::Vector3f SlamProcessor::getLastScanMatchPose() const
{
	return lastScanMatchPose;
}*/

const Eigen::Vector3f SlamProcessor::getLastScanMatchPose() const
{
        return lastScanMatchPose;
}

/*Eigen::Vector3f SlamProcessor::getLastMapUpdatePose() const
{
	return lastMapUpdatePose;
}*/

const Eigen::Vector3f SlamProcessor::getLastMapUpdatePose() const
{
        return lastMapUpdatePose;
}


}



























