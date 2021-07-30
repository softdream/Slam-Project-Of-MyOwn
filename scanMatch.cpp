#include "scanMatch.h"
#include "utils.h"

namespace slam {

ScanMatchMethod::ScanMatchMethod() : mP00( 0 ), 
				     mP11( 0 ),
				     mP01( 0 ),
				     mP10(0)
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


void ScanMatchMethod::getHessianDerivative( const OccupiedMap &occuMap,
					    Eigen::Vector3f &robotPoseInWorld,
                                   	    const ScanContainer &scanPoints, 
                                   	    Eigen::Matrix3f &H,  
                                   	    Eigen::Vector3f &dTr )
{
	int size = scanPoints.getSize();

	float sinRot = ::sin( robotPoseInWorld[2] );
	float cosRot = ::cos( robotPoseInWorld[2] );

	H = Eigen::Matrix3f::Zero();
	dTr = Eigen::Vector3f::Zero();	

	for( int i = 0; i < size; i ++ ){
		// 1. get the current point in laser coordinate
		Eigen::Vector2f currPointInLaser( scanPoints.getIndexData( i ) );
		Eigen::Vector2f currPointInScaleLaser( scanPoints.getIndexData( i ) * occuMap.getScale() );	
	
		// 2. Transform the End Point from Laser Coordinate to World Coordinate
		Eigen::Vector2f currPointInWorld( occuMap.observedPointPoseLaser2World( currPointInLaser, robotPoseInWorld ) );
		
		// 3. Transform the End Point from World Coordinate to Map Coordinate
		Eigen::Vector2f currPointInMap( occuMap.observedPointPoseWorld2Map( currPointInWorld ) );

		// 4. get the M(Pm), d(M(Pm))/dx, d(M(Pm))/dy
		Eigen::Vector3f interpolatedValue( bilinearInterpolationWithDerivative( occuMap, currPointInMap ) );
		
		// 5. the Objective Function: f(x) = 1 - M(Pm)
		float funcValue = 1.0f - interpolatedValue[0];
	
		// 6. 
		dTr[0] += interpolatedValue[1] * funcValue;
		dTr[1] += interpolatedValue[2] * funcValue;

		// 7. 
		float rotDeriv = ( interpolatedValue[1] * ( -sinRot * currPointInScaleLaser[0] - cosRot * currPointInScaleLaser[1] ) ) + ( interpolatedValue[2] * ( cosRot * currPointInScaleLaser[0] - sinRot * currPointInScaleLaser[1] ) );

		// 8. 
		dTr[2] += rotDeriv * funcValue;

		// 9. 
		H( 0, 0 ) += sqr( interpolatedValue[1] );
		H( 1, 1 ) += sqr( interpolatedValue[2] );
		H( 2, 2 ) += sqr( rotDeriv );

		H( 0, 1 ) += interpolatedValue[1] * interpolatedValue[2];
		H( 0, 2 ) += interpolatedValue[1] * rotDeriv;
		H( 1, 2 ) += interpolatedValue[2] * rotDeriv;

	}

	// 10. 
	H( 1, 0 ) = H( 0, 1 );
	H( 2, 0 ) = H( 0, 2 );
	H( 2, 1 ) = H( 1, 2 );
}

bool ScanMatchMethod::estimateTransformationOnce( const OccupiedMap &occuMap, 
                                 Eigen::Vector3f &estimateInWorld,  
                                 const ScanContainer &scanPoints )
{
	getHessianDerivative( occuMap, estimateInWorld, scanPoints, H, dTr );
	
	//std::cout<<"Hessian : "<<std::endl;
	//std::cout<<H<<std::endl;
	//std::cout<<"dTr: "<<std::endl;
	//std::cout<<dTr<<std::endl;;

	if ( ( H(0, 0) != 0.0f ) && ( H(1, 1) != 0.0f ) ){
		Eigen::Vector3f deltaCauchy( H.inverse() * dTr );
		
		if( deltaCauchy[2] > 0.2f ){
			deltaCauchy[2] = 0.2f;
			std::cout<<"delta Cauchy angle change too large"<<std::endl;
		}
		else if( deltaCauchy[2] < -0.2f ){
			deltaCauchy[2] = -0.2f;
			std::cout<<"delta Cauchy angle change too small"<<std::endl;
		}
	
		updateEstimatedPose( occuMap, estimateInWorld, deltaCauchy );
		
		return true;
	}
	
	return false;
	
}

void ScanMatchMethod::updateEstimatedPose( const OccupiedMap &occuMap, Eigen::Vector3f &estimateInWorld, Eigen::Vector3f &delta )
{
	Eigen::Vector3f estimateInMap( occuMap.robotPoseWorld2Map( estimateInWorld ) );

	estimateInMap += delta;
	
	estimateInWorld = occuMap.robotPoseMap2World( estimateInMap );
}

Eigen::Vector3f ScanMatchMethod::scanToMap( const OccupiedMap &occuMap, 
                        Eigen::Vector3f &beginEstimatedPoseInWorld, 
                        const ScanContainer &scanPoints, 
                        Eigen::Matrix3f &covarinceMatrix, 
                        int maxInterations )
{	
	Eigen::Vector3f estimatePose( beginEstimatedPoseInWorld );

	if( scanPoints.getSize() == 0 ){
		return beginEstimatedPoseInWorld;
	}		
	
	// 1. first iteration
	estimateTransformationOnce( occuMap, estimatePose, scanPoints );
	
	// 2. multiple iterations
	for( int i = 0; i < maxInterations - 1; i ++ ){
		estimateTransformationOnce( occuMap, estimatePose, scanPoints );
			
	}
	
	// 3. normalize the angle [-PI ~ PI]
	estimatePose[2] = normalize_angle( estimatePose[2] );

	// 4. get the covariance matrix
	covarinceMatrix = Eigen::Matrix3f::Zero();

	covarinceMatrix = H;
	
	// 5. return the estimated pose in world coordinate
	return estimatePose;
}




}















