#ifndef __LOOP_CLOSURE_SCAN_CONTEXT_H_
#define __LOOP_CLOSURE_SCAN_CONTEXT_H_

#include "loopClosureBase.h"
#include "scanContext.h"

namespace slam{

class ScanContextLoopClosure : public LoopClosureBase
{
public:
	ScanContextLoopClosure();
	virtual ~ScanContextLoopClosure();

	virtual void detectLoop( const slam::sensor::LaserScan &scan );
	virtual void caculateTransformByICP();

private:
	ICP icp;	

	ScanContext<float, 20> scanContext;
	std::vector<slam::sensor::LaserScan> scanVec;
	
	int matchedScanID = -1;
	float angleBias = 0.0f;
	
};



}



#endif


