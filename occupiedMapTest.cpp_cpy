#include "occupiedGridMap.h"
#include "gridBase.h"

#include <opencv2/opencv.hpp>

#include "scanContainer.h"

#include "utils.h"

int main()
{
	std::cout<<"Occupied Grid Map Test ..."<<std::endl;

	slam::OccupiedGridMap<slam::GridBase> occumap;

	cv::Mat image = cv::Mat::zeros(occumap.getSizeX(), occumap.getSizeY(), CV_8UC3);
	cv::Point2d center( occumap.getMapCenter()[0], occumap.getMapCenter()[1] );
	std::cout<<"center: ( "<<occumap.getMapCenter()[0]<<", "<<occumap.getMapCenter()[1]<<" )"<<std::endl;
	cv::circle(image, center, 3, cv::Scalar(0, 0, 255), 3);

	cv::circle(image, cv::Point2d( 200, 200 ), 1, cv::Scalar(0, 0, 255), 1);
	
	slam::ScanContainer laserPoints;
	
	double theta = -std::fabs(-3.12144 - 3.14159) / 360;
	for( int i = 0; i < 360; i ++ ){
		
		float dist = 30.0f;
		Eigen::Vector2f point( ::cos( theta ) * dist, ::sin( theta ) * dist );
		std::cout<<"laser point: ( "<<point[0]<<", "<<point[1]<<" )"<<std::endl;
		laserPoints.addData( point );

		theta += std::fabs(0.0174533);
	}
	
	Eigen::Vector3f robotPose( 0.0f, 0.0f, 0.0f );
	occumap.updateByScan_test( laserPoints, robotPose );

	std::cout<<"----------------- Get the Map ----------------"<<std::endl;
	
	int occupiedCount = 0;
	for( int i = 0; i < occumap.getSizeX(); i ++ ){
		for( int j = 0; j < occumap.getSizeY(); j ++ ){
			if( occumap.isCellFree( i, j ) ){
				cv::circle(image, cv::Point2d(i, j), 1, cv::Scalar(255, 255, 255), 1);
				//std::cout<<"Free Point: ( "<<i<<", "<<j<<" )"<<std::endl;
                                //std::cout<<"prob: "<<occumap.getCellOccupiedProbability( i, j )<<std::endl;
			}
			else if( occumap.isCellOccupied( i, j ) ){
				occupiedCount ++;
				std::cout<<"Occupied Point: ( "<<i<<", "<<j<<" )"<<std::endl;
				std::cout<<"prob: "<<occumap.getCellOccupiedProbability( i, j )<<std::endl;
				std::cout<<"log Odds value: "<<occumap.getCellLogOdds(i, j)<<std::endl;
				cv::circle(image, cv::Point2d(i, j), 1, cv::Scalar(0, 0, 255), 1);
			}
		}
	}

	std::cout<<"---------------- Result --------------------"<<std::endl;
	std::cout<<"Occupied Points Number: "<<occupiedCount<<std::endl;
	
	cv::imshow( "test", image );	
	
	
	cv::waitKey(0);
	return 0;
}
