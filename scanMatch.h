#ifndef __SCAN_MATCH_H_
#define __SCAN_MATCH_H_

#include <cmath>
#include <Eigen/Dense>
#include "occupiedMap.h"

namespace slam {

class ScanMatchMethod
{
public:
	ScanMatchMethod();
	~ScanMatchMethod();

	float bilinearInterpolation( const OccupiedMap &occuMap, const Eigen::Vector2f &coords );	

	Eigen::Vector3f bilinearInterpolationWithDerivative( const OccupiedMap &occuMap, const Eigen::Vector2f &coords );
	

	void getHessianDerivative( const Eigen::Vector3f robotPoseInMap,
				   const ScanContainer &scanPoints,
				   Eigen::Matrix3f &H, 
				   Eigen::Vector3f &dTr );

private:
	
	float mP00;
	float mP11;
	float mP01;
	float mP10;
};


}


#endif
