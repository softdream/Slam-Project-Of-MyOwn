#include "icp.h"
#include <cmath>

namespace slam{

ICP::ICP()
{

}

ICP::~ICP()
{

}

const float ICP::operator()( ScanContainer& A_src, ScanContainer& B_src )
{
	std::vector<Eigen::Vector2f> B; 
	std::vector<Eigen::Vector2f> B_apostrophe( B_src.getSize(), Eigen::Vector2f( 0.0f, 0.0f ) );	

	for( size_t i = 0; i < A_src.getSize(); i ++ ){
		A.push_back( A_src.getIndexData(i) );
	}	
	
	for( size_t i = 0; i < B_src.getSize(); i ++ ){
                B.push_back( B_src.getIndexData(i) );
        }

	// 1. Caculate the center of point cloud of A
	Eigen::Vector2f Asum(0.0f, 0.0f);
	for( auto it : A ){
		Asum += it;
	}
	Acenter = Asum / A.size();
	// 2. Caculate the distance from points to the center of A
	for( auto it : A ){
		Eigen::Vector2f tmp = it - Acenter;
		a.push_back( tmp );
	}
	
	// begin iterate
	int iteration = 0;
	float loss = 0.0f;
	while( iteration < maxIteration ){
		loss = iterateOnce( B, B_apostrophe );
		
		B.swap( B_apostrophe );	
	}

	return loss;
}

float ICP::iterateOnce( std::vector<Eigen::Vector2f>& B, std::vector<Eigen::Vector2f>& B_apostrophe )
{
	Eigen::Vector2f Bsum(0.0f, 0.0f);

	for( auto it : B ){
		Bsum += it;
	}
	Bcenter = Bsum / B.size();

	for( auto it : B ){
		Eigen::Vector2f tmp = it - Bcenter;
		b.push_back( tmp );
	}

	// 3. caculate the rotate theta
	float y = 0.0f, x = 0.0f;
	for( size_t i = 0; i < b.size(); i ++ ){
		y += ( a[i](0) * b[i](1) ) - ( a[i](1) * b[i](0) );
	
		x += ( a[i](0) * b[i](0) ) + ( a[i](1) * b[i](1) );
	}
	float theta = ::atan2( y, x );
	
	// 4. get the rotate matrix and transfrom matrix
	R( 0, 0 ) = ::cos(theta);
	R( 0, 1 ) = -::sin(theta);
	R( 1, 0 ) = ::sin(theta);
	R( 1, 1 ) = ::cos(theta);
	
	T = Acenter - R * Bcenter;

	// 5. get new pose B' from B
	for( size_t i = 0; i < B.size(); i ++ ){
		B_apostrophe[i] = R * B[i] + T;
	}

	// 6. caculate loss function
	float loss = 0.0f;
	for( size_t i = 0; i < B.size(); i ++ ){
		Eigen::Vector2f tmp = A[i] - B[i];
		loss += tmp.squaredNorm();
	}
	loss /= B.size();

	return loss;
}


}






