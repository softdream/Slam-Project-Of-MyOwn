#include "slamProcessor.h"
#include "laserSimulation.h"

#include "odomSimulation.h"

#include "loopClosureScanContext.h"

#include "graphOptimize.h"

#include <unistd.h>

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

bool poseDiffLargerThan( Eigen::Vector3f &poseOld, Eigen::Vector3f &poseNew )
{

        if( ( ( poseNew.head<2>() - poseOld.head<2>() ).norm() ) > 0.4f ){
                return true;
        }

        float angleDiff = ( poseNew.z() - poseOld.z() );

        if( angleDiff > M_PI ){
                angleDiff -= M_PI * 2.0f;
        }
        else if( angleDiff < -M_PI ){
                angleDiff += M_PI * 2.0f;
        }

        if( ::abs( angleDiff ) > 0.9f ){
                return true;
        }

        return false;
}


int main()
{
	std::cout<<"--------------------- G2O Test --------------------"<<std::endl;
	
	slam::simulation::OdomSimulation odomSim;

	// g2o instance
        slam::optimizer::GraphOptimize optimizer;

	optimizer.createOptimizer();

	std::string odom_file_name = "../../../../simulation_file/odometry2.txt";
 	if( !odomSim.openSimulationFile( odom_file_name ) ){
		//std::cout<<"Open File Failed ..."<<std::endl;
		//return 0;
	}

	Eigen::Vector3f poseOld = Eigen::Vector3f::Zero();
        Eigen::Vector3f poseNew = poseOld;

        std::vector<Eigen::Vector3f> keyPoses;

	int keyCount = -1;


	while( !odomSim.endOfInputFile() ){
		odomSim.readAFrameData( poseNew );
		std::cout<<"frame count: "<<odomSim.getFrameCount()<<std::endl;
  //              std::cout<<"pose: "<<std::endl<<poseNew<<std::endl;

		if( poseDiffLargerThan( poseOld, poseNew ) ){
                        std::cerr<<"------------------ UPDATE ----------------"<<std::endl;
			std::cout<<"pose: "<<std::endl<<poseNew<<std::endl;
			keyCount ++;	

			if( keyCount == 0 ){
				std::cout<<"keyCount = "<<keyCount <<std::endl;
                                keyPoses.push_back( poseNew );

				optimizer.addVertex( poseNew, keyCount ); // add a vertex
			}
			else{
        	               std::cout<<"keyCount = "<<keyCount <<std::endl;
                	        keyPoses.push_back( poseNew );
				
				optimizer.addVertex( poseNew, keyCount ); // add a vertex

				Eigen::Vector3f poseDiff = poseNew - poseOld;
				std::cout<<"Pose Diff: "<<std::endl<<poseDiff<<std::endl;				

				Eigen::Matrix3d information = 10 * Eigen::Matrix3d::Identity(); //information matrix
				//std::cout<<"information matrix: "<<std::endl<<information<<std::endl;
		
                	        optimizer.addEdge( poseDiff, keyCount - 1, keyCount, information ); // add a edge
			}

			if( keyCount % 130 == 0 && keyCount != 0 ){
				std::cout<<"----------- execuate the graph optimization ----------"<<std::endl;
                        	// execuate the graph optimization every 50 key points
				
                                optimizer.execuateGraphOptimization();

                                optimizer.getOptimizedResults();

                                std::vector<Eigen::Vector3f> estimatedPoses = optimizer.getEstimatedPoses();
                                std::cout<<"keyPoses.size  = "<<keyPoses.size()<<std::endl;
                                std::cout<<"estimatedPoses.size = "<<estimatedPoses.size()<<std::endl;
                                // TODO ... process the estimated results

				break;
                        }


			poseOld = poseNew;
		}		
		
		cv::waitKey( 60 );
	}

	odomSim.closeSimulationFile();

	return 0;
}
