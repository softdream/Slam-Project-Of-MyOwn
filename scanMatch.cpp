#include "scanMatch.h"

namespace slam {

ScanMatchMethod::ScanMatchMethod()
{

}

ScanMatchMethod::~ScanMatchMethod()
{

}

float ScanMatchMethod::bilinearInterpolation( const OccupiedMap &occuMap, const Eigen::Vector2f &coords )
{
	// 1. judge weather out of range
	if( occuMap.isPointOutOfRange( coords ) ){
		return 0.0f;
	}		

	// 2. map coords are always positive, floor them by casting to int
	Eigen::Vector2i indMin( coords.cast<int>() );

	// 3. factor0 = ( x - x0 )
	//    factor1 = ( y - y0 )
	float factor0 = coords[0] - static_cast<float>( indMin[0] );
	float factor1 = coords[1] - static_cast<float>( indMin[1] );

	// 4. find p(m) point in map coordinate
	int sizeX = occuMap.getSizeX();
	int index = indMin[1] * sizeX + indMin[0];

	// 5. get the probability of the four points
	mP00 = occuMap.getCellOccupiedProbability( index );

	index ++;
	mP10 = occuMap.getCellOccupiedProbability( index );

	index += sizeX - 1;
	mP01 = occuMap.getCellOccupiedProbability( index );

	index ++;
	mP11 = occuMap.getCellOccupiedProbability( index );

	// 6. factorInv0 = 1 - ( x - x0 )
	//    factorInv1 = 1 - ( y - y0 )
	float factorInv0 = 1.0f - factor0;
	float factorInv1 = 1.0f - factor1; 

	// 7. M(Pm) = (y - y0) * { (x - x0) * M(P11) + [1 - (x - x0)] * M(P01) } + [1 - (y - y0)] * { (x - x0) * M(P10) + [1 - (x - x0)] * M(P00) }
	return ( factor1 * ( factor0 * mP11 + factorInv0 * mP01 ) ) + ( factorInv1 * ( factor0 * mP10 + factorInv0 * mP00 ) );
}


Eigen::Vector3f ScanMatchMethod::bilinearInterpolationWithDerivative( const OccupiedMap &occuMap, const Eigen::Vector2f &coords )
{
	// 1. judge weather out of range
        if( occuMap.isPointOutOfRange( coords ) ){
                return Eigen::Vector3f( 0.0f, 0.0f, 0.0f );
        }

        // 2. map coords are always positive, floor them by casting to int
        Eigen::Vector2i indMin( coords.cast<int>() );

        // 3. factor0 = ( x - x0 )
        //    factor1 = ( y - y0 )
        float factor0 = coords[0] - static_cast<float>( indMin[0] );
        float factor1 = coords[1] - static_cast<float>( indMin[1] );

        // 4. find p(m) point in map coordinate
        int sizeX = occuMap.getSizeX(); 
        int index = indMin[1] * sizeX + indMin[0];

        // 5. get the probability of the four points
        mP00 = occuMap.getCellOccupiedProbability( index );

        index ++;
        mP10 = occuMap.getCellOccupiedProbability( index );

        index += sizeX - 1;
        mP01 = occuMap.getCellOccupiedProbability( index );

        index ++;
        mP11 = occuMap.getCellOccupiedProbability( index );

        // 6. factorInv0 = 1 - ( x - x0 )
        //    factorInv1 = 1 - ( y - y0 )
        float factorInv0 = 1.0f - factor0;
        float factorInv1 = 1.0f - factor1;

	
	// 7. M(Pm) = (y - y0) * { (x - x0) * M(P11) + [1 - (x - x0)] * M(P01) } + [1 - (y - y0)] * { (x - x0) * M(P10) + [1 - (x - x0)] * M(P00) }
	// ---------------------------------------------------------------------------------------------------------------------------------------
	// d(M(Pm)) / dx = (y - y0) * [M(P11) - M(P01)] + (1 - (y - y0)) * [M(P10) - M(P00)]
	// ---------------------------------------------------------------------------------
	// d(M(Pm)) / dy = (x - x0) * [M(P11) - M(P10)] + (1 - (x - x0)) * [M(P01) - M(P00)]
	return Eigen::Vector3f( ( ( factor1 * ( factor0 * mP11 + factorInv0 * mP01 ) ) + ( factorInv1 * ( factor0 * mP10 + factorInv0 * mP00 ) ) ), 
				( factor1 * ( mP11 - mP01 ) + factorInv1 * ( mP10 - mP00 ) ),
				( factor0 * ( mP11 - mP10 ) + factorInv0 * ( mP01 - mP00 ) ) 
			      );
}


void ScanMatchMethod::getHessianDerivative( const Eigen::Vector3f robotPoseInMap, 
                                   const ScanContainer &scanPoints, 
                                   Eigen::Matrix3f &H,  
                                   Eigen::Vector3f &dTr )
{

}


}
