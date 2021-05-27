#include "occupiedGridMap.h"
#include "gridBase.h"

#include <opencv2/opencv.hpp>

int main()
{
	std::cout<<"Occupied Grid Map Test ..."<<std::endl;

	slam::OccupiedGridMap<slam::GridBase> occumap;

	cv::Mat image = cv::Mat::zeros(occumap.getSizeX(), occumap.getSizeY(), CV_8UC3);
	cv::Point2d center( occumap.getMapCenter()[0], occumap.getMapCenter()[1] );
	std::cout<<"center: ( "<<occumap.getMapCenter()[0]<<", "<<occumap.getMapCenter()[1]<<" )"<<std::endl;
	cv::circle(image, center, 3, cv::Scalar(0, 0, 255), 3);

	occumap.updateByScan_test( );
	
	/*for( int i = 0; i <  10;  i ++ ){
		for( int j = 0; j < 10; j ++ ){
			std::cout<<"log Odds Value: "<<occumap.getCellLogOdds(i, j)<<std::endl;
		
		}
	}*/

	for( int i = 0; i < occumap.getSizeX(); i ++ ){
		for( int j = 0; j < occumap.getSizeY(); j ++ ){
			if( occumap.isCellFree( i, j ) ){
				cv::circle(image, cv::Point2d(i, j), 3, cv::Scalar(255, 255, 255), 3);
				std::cout<<"Free Point: ( "<<i<<", "<<j<<" )"<<std::endl;
                                std::cout<<"prob: "<<occumap.getCellOccupiedProbability( i, j )<<std::endl;
			}
			else if( occumap.isCellOccupied( i, j ) ){
				std::cout<<"Occupied Point: ( "<<i<<", "<<j<<" )"<<std::endl;
				std::cout<<"prob: "<<occumap.getCellOccupiedProbability( i, j )<<std::endl;
				std::cout<<"log Odds value: "<<occumap.getCellLogOdds(i, j)<<std::endl;
				cv::circle(image, cv::Point2d(i, j), 3, cv::Scalar(0, 0, 255), 3);
			}
		}
	}
	
	cv::imshow( "test", image );	
	
	
	cv::waitKey(0);
	return 0;
}
