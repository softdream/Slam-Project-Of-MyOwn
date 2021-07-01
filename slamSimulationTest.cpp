#include "slamProcessor.h"
#include "laserSimulation.h"

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
	// open the simulation file
	std::string file_name = "";
	simulation.openSimulationFile( file_name );

	// convarince
	Eigen::Matrix3f covarince;

	// robot pose
	Eigen::Vector3f robotPosePrev( 0.0f, 0.0f, 0.0f );
//	Eigen::Vector3f robotPoseCurr( 0.0f, 0.0f, 0.0f );

	// slam process
	// while it is not the end of the simulation file
	while( !simulation.endOfFile() ){
		// 1. get the laser data
		slam::sensor::LaserScan scan;
		slam::ScanContainer scanContainer;
		simulation.readAFrameData( scan ); // read the laser data
		
		slam.laserData2Container( scan, scanContainer );// convert the laser data to scanContainer type	
	
		// 2. if this is the first frame of laser data
		if( simulation.getFrameCount() == 1 ){
			//Eigen::Vector3f robotPose( 0.0f, 0.0f, 0.0f );
			// Process the first laser scan
			slam.processTheFirstScan( robotPosePrev, scanContainer );

			// display the map
		}	
	
		// 3. Update by Scan Match, get the estimated pose 
		slam.update( robotPosePrev, scanContainer );
		
		// 4. get the newest robot pose in world coordinate
		robotPosePrev = slam.getLastScanMatchPose();

		// 4. display the map
	}
		

	// close the simulation file
	simulation.closeSimulationFile();

	return 0;
}
