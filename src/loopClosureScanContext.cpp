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
	
}

}
