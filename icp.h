#ifndef __ICP_H_
#define __ICP_H_

#include <Eigen/Dense>
#include "scanContainer.h"
#include <vector>

namespace slam{

class ICP{
public:
	ICP();
	~ICP();

	const float operator()( ScanContainer& A_src, ScanContainer& B_src );

private:
	float iterateOnce( std::vector<Eigen::Vector2f>& B, std::vector<Eigen::Vector2f>& B_apostrophe );

private:
	Eigen::Vector2f Acenter;
	Eigen::Vector2f Bcenter;

	std::vector<Eigen::Vector2f> A;
//        std::vector<Eigen::Vector2f> B;	

	std::vector<Eigen::Vector2f> a;
	std::vector<Eigen::Vector2f> b;	

	Eigen::Matrix<float, 2, 2> R;
	Eigen::Vector2f T;
	
	int maxIteration = 100;
};

}


#endif
