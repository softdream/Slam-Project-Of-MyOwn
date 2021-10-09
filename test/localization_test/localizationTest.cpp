#include <unistd.h>
#include "laserSimulation.h"

#include "loadMap.h"

#include "localization.h"

#include "slamProcessor.h"


void laserData2Container( const slam::sensor::LaserScan &scan, slam::ScanContainer &container )
{
        size_t size = 1440;

        float angle = -3.12413907051f;
        container.clear();

        for( int i = 0; i < size; i ++ ){ 
                float dist = scan.ranges[ i ];

                if( dist >= 0.25f && dist <= 15.0000000000f ){
                        //dist *= scaleToMap;
                        container.addData( Eigen::Vector2f( cos(angle) * dist, sin(angle) * dist ) );
                }

                angle += 0.00435422640294f;
        }

        std::cout<<"Scan Container Size: "<<container.getSize()<<std::endl;
}


void displayMap( slam::OccupiedMap &occupiedGridMap, cv::Mat &image )
{

        int occupiedCount = 0;

        // display the map
        for( int i = 0; i < occupiedGridMap.getSizeX(); i ++ ){
                for( int j = 0; j < occupiedGridMap.getSizeY(); j ++ ){
                        if( occupiedGridMap.isCellFree( i, j ) ){
                                cv::circle(image, cv::Point2d(i, j), 1, cv::Scalar(255, 255, 255), -1);

                        }
                        else if( occupiedGridMap.isCellOccupied( i, j ) ){
                                occupiedCount ++;
                                cv::circle(image, cv::Point2d(i, j), 1, cv::Scalar(0, 0, 255), -1);
                        }

                }
        }
 
        std::cout<<"---------------- Result --------------------"<<std::endl;
        std::cout<<"Occupied Points Number: "<<occupiedCount<<std::endl;
}

/*void dispalyScans( slam::OccupiedMap &occupiedGridMap, slam::ScanContainer &container, cv::Mat &image )
{
	for( size_t i = 0; i < container.size(); i ++ ){
		Eigen::Vector2f pointInWorld;
		occupiedGridMap.observedPointPoseWorld2Map(  );
	}
}*/


int main()
{
	std::cout<<"--------------------- Load Map Test --------------------"<<std::endl;

	// 1. instance of the OccupiedMap
	slam::OccupiedMap occupiedMap;
	
	slam::simulation::Simulation simulation;

	// 2. read the Occupied Map from the file
	occupiedMap = slam::LoadMap()( "../../../simulation_file/test.map" );
	occupiedMap.setMapInfo( 1001,  1001, 10);

	// open the simulation file
        std::string file_name = "../../../simulation_file/laser_data2.txt";
        simulation.openSimulationFile( file_name );

	
	// 3. print the information of the map
	std::cout<<"-------------------- Map Information -----------------"<<std::endl;
	std::cout<<"map sizeX : "<<occupiedMap.getSizeX()<<std::endl;	
	std::cout<<"map sizeY : "<<occupiedMap.getSizeY()<<std::endl;
	std::cout<<"map cell length : "<<occupiedMap.getCellLength()<<std::endl;
	std::cout<<"------------------------- END -----------------------"<<std::endl;
	
	// init the image
	cv::Mat image = cv::Mat::zeros(occupiedMap.getSizeX(), occupiedMap.getSizeY(), CV_8UC3);

	// display the map
	displayMap( occupiedMap, image );
	cv::imshow( "map", image );
	cv::waitKey(0);

	slam::SlamProcessor slam;

 	// robot pose
        Eigen::Vector3f robotPose( 0.001f, 0.01f, 0.001f );
	

	while( !simulation.endOfFile() ){
                // 1. get the laser data
                slam::sensor::LaserScan scan;

                slam::ScanContainer scanContainer;
                simulation.readAFrameData( scan ); // read the laser data

                laserData2Container( scan, scanContainer );// convert the laser data to scanContainer type
		scanContainer.displayAFrameScan();
	
		std::cout<<"frame count: "<<simulation.getFrameCount()<<std::endl;

		slam.update( robotPose, scanContainer );
		robotPose = slam.getLastScanMatchPose();
                std::cout<<"robot pose now: "<<std::endl;
                std::cout<<robotPose<<std::endl;
                std::cout<<"------------------"<<std::endl;
		
		cv::waitKey(60);
	}
	
	simulation.closeSimulationFile();
	return 0;
}
