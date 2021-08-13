#include "loopClosureScanContext.h"

namespace slam{

ScanContextLoopClosure::ScanContextLoopClosure()
{

}

ScanContextLoopClosure::~ScanContextLoopClosure()
{

}

void ScanContextLoopClosure::detectLoop( const slam::sensor::LaserScan &scan )
{
	scanVec.push_back( scan );

	scanContext.makeAndSaveScancontextAndKeys( scan );

	std::pair<int, float> ret = scanContext.detectLoopClosureID();

	matchedScanID = ret.first;
	angleBias = ret.second;

}

void ScanContextLoopClosure::caculateTransformByICP()
{
	ScanContainer pointsCandidate;
	ScanContainer pointsNow;

	sensor::LaserScan scanNow = scanVec.back();
	sensor::LaserScan scanCandidate = scanVec[ matchedScanID ];

	pointsCandidate.pointTransform2LaserCoords( scanCandidate ) ;
	pointsNow.pointTransform2LaserCoords( scanNow );

	float loss = icp.solveICP( pointsCandidate, pointsNow );
	std::cout<<"loss = "<<loss<<std::endl;
	
}

const Eigen::Matrix<float,2 ,2> ScanContextLoopClosure::getRotateMatrix() const 
{
	return icp.getRotateMatrix();
}

const Eigen::Vector2f ScanContextLoopClosure::getTransformVector() const
{
	return icp.getTransform();
}

void ScanContextLoopClosure::setPose( const Eigen::Vector3f &pose )
{

}

}
