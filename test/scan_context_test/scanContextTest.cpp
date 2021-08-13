#include <iostream>
#include "laserSimulation.h"

#include "scanContext.h"
#include "scanContainer.h"

void laserData2Container( const slam::sensor::LaserScan &scan, slam::ScanContainer &container )
{
        size_t size = 1440;

        float angle = -3.14159f;
        container.clear();

        for( int i = 0; i < size; i ++ ){ 
                float dist = scan.ranges[ i ];

                if( dist >= 0.0099999998f && dist <= 15.0000000000f ){
                        //dist *= scaleToMap;
                        container.addData( Eigen::Vector2f( cos(angle) * dist, sin(angle) * dist ) );
                }

                angle += 0.0043633231f;
        }

        std::cout<<"Scan Container Size: "<<container.getSize()<<std::endl;
}


int main()
{
	std::cout<<" ------------- Scan Context Test ---------------"<<std::endl;
	// ScanContext instance
	slam::ScanContext<float, 20> scanContext;
	slam::simulation::Simulation simulation;
	
	// open the simulation file
        std::string file_name = "laser_data.txt";
        simulation.openSimulationFile( file_name );

	// LaserScan instance & ScanContainer instance
	slam::sensor::LaserScan scan;
	slam::ScanContainer scanContainer;

	while( !simulation.endOfFile() ){	

	// read a frame of data
		simulation.readAFrameData( scan );	
		std::cout<<"frame count: "<<simulation.getFrameCount()<<std::endl;

		laserData2Container( scan, scanContainer );// convert the laser data to scanContainer type
		scanContainer.displayAFrameScan( 20.0f );

		Eigen::MatrixXf sc = scanContext.makeScanContext( scan );
		scanContext.displayAScancontext( sc );
		//scanContext.makeRingkeyFromScancontext( sc );
		
	//	scanContext.makeAndSaveScancontextAndKeys( scan );	
	//	scanContext.detectLoopClosureID();	
	
		cv::waitKey( 60 );	
	
	}
		
	simulation.closeSimulationFile();

	return 0;
}
