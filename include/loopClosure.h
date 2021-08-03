#ifndef __LOOP_CLOSURE_H_
#define __LOOP_CLOSURE_H_

#include "icp.h"
#include "scanContainer.h"

namespace slam{

class LoopClosure{
public:
	LoopClosure();
	~LoopClosure();

	bool operator()( const ScanContainer &scan1, const ScanContainer &scan2 );

private:
	ICP icp;

};

}

#endif
