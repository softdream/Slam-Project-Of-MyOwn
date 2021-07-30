#include "slamProcessor.h"
#include "laserSimulation.h"
#include <unistd.h>

int main()
{
	std::cout<<"--------------------- SLAM Simulation --------------------"<<std::endl;
	
	// slam classes instances
	slam::SlamProcessor slam;
	slam::simulation::Simulation simulation;
	slam::MapInfo mapInfo = slam.getMapInfo();
	
	// print the map information
	std::cout<<"------------- Map Information ----------------"<<std::endl;
	std::cout<<"Map SizeX: "<<mapInfo.getSizeX()<<std::endl;
	std::cout<<"Map SizeY: "<<mapInfo.getSizeY()<<std::endl;
	std::cout<<"Map Center: ( "<<mapInfo.getMapCenter()[0]<<", "<<mapInfo.getMapCenter()[1]<<" )"<<std::endl;
	std::cout<<"Map Scale: "<<mapInfo.getScale()<<std::endl;
	std::cout<<"Map Cell Length: "<<mapInfo.getCellLength()<<std::endl;
	std::cout<<"----------------------------------------------"<<std::endl;		
	
	// init the image
	cv::Mat image = cv::Mat::zeros(mapInfo.getSizeX(), mapInfo.getSizeY(), CV_8UC3);
        cv::Point2d center( mapInfo.getMapCenter()[0], mapInfo.getMapCenter()[1] );
        std::cout<<"center: ( "<<mapInfo.getMapCenter()[0]<<", "<<mapInfo.getMapCenter()[1]<<" )"<<std::endl;
        cv::circle(image, center, 3, cv::Scalar(0, 0, 255), 3);
	cv::imshow("map", image);
	
	// open the simulation file
	std::string file_name = "laser_data2.txt";
	simulation.openSimulationFile( file_name );

	// convarince
	Eigen::Matrix3f covarince;

	// robot pose
	Eigen::Vector3f robotPosePrev( 0.0f, 0.0f, 0.0f );
//	Eigen::Vector3f robotPoseCurr( 0.0f, 0.0f, 0.0f );

	// slam process
	// while it is not the end of the simulation file
	int count = 0;
	while( !simulation.endOfFile() ){
//	while( count < 21 ){
		// 1. get the laser data
		slam::sensor::LaserScan scan;
		scan.angle_min = -3.12414f;
		scan.angle_max = 3.14159f;
		scan.angle_increment = 0.0174532924f;
		scan.range_min = 0.1500000060f;
		scan.range_max = 12.0000000000f;
	
		slam::ScanContainer scanContainer;
		simulation.readAFrameData( scan ); // read the laser data
		
		slam.laserData2Container( scan, scanContainer );// convert the laser data to scanContainer type	
	
		std::cout<<"frame count: "<<simulation.getFrameCount()<<std::endl;	

		// 2. if this is the first frame of laser data
		if( simulation.getFrameCount() < 20 ){
			//Eigen::Vector3f robotPose( 0.0f, 0.0f, 0.0f );
			// Process the first laser scan
			slam.processTheFirstScan( robotPosePrev, scanContainer );
			std::cout<<"---------------- Process The First Laser Scan --------------------"<<std::endl;		
	
			// display the map
			slam.displayMap( image );
		}
		else{	
			// 3. Update by Scan Match, get the estimated pose 
			slam.update( robotPosePrev, scanContainer );
		
			// 4. get the newest robot pose in world coordinate
			robotPosePrev = slam.getLastScanMatchPose();
			std::cout<<"robot pose now: "<<std::endl;
			std::cout<<robotPosePrev<<std::endl;
			std::cout<<"------------------"<<std::endl;
		
			// 4. display the map
			slam.displayMap( image );
		}
		cv::waitKey(1000);	
		count ++;
	}
		

	// close the simulation file
	simulation.closeSimulationFile();


	return 0;
}
