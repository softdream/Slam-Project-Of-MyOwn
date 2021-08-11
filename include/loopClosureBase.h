#ifndef __LOOP_CLOSURE_BASE_H_
#define __LOOP_CLOSURE_BASE_H_

#include "dataType.h"
#include "scanContainer.h"
#include "icp.h"

namespace slam{

class LoopClosureBase{
public:
	LoopClosureBase()
	{
	
	}
	
	virtual ~LoopClosureBase()
	{

	}

	virtual void detectLoop( const slam::sensor::LaserScan &scan ) = 0;
	virtual void caculateTransformByICP() = 0;
	
};

}

#endif
