#include <iostream>
#include <icp.h>
#include "scanContainer.h"
#include "laserSimulation.h"
#include "dataType.h"

void laserData2Container( const slam::simulation::Laser &scan, slam::ScanContainer &container )
{
	container.clear();

	double theta = -std::fabs(-3.12144 - 3.14159) / 360;
		
	for( int i = 0; i < 360; i ++ ){
		float dist = scan.range[i];

		if( dist > 0.1f && dist < 10.0f ){
			Eigen::Vector2f point(::cos(theta) * dist, ::sin(theta) * dist);
			std::cout<<"laser point: ( "<<point[0]<<", "<<point[1]<<" )"<<std::endl;
			container.addData(point);
		}
	
		theta += std::fabs( 0.0174533f );
	}
	
	std::cout<<"Scan Container Size: "<<container.getSize()<<std::endl;
}

int main()
{
  std::cout<<"-----------------------  ICP TEST ------------------------"<<std::endl;
  slam::simulation::Laser scanArray[2];
  
  slam::ScanContainer laserPoints[2];
	slam::simulation::readLaserTXT( scanArray );
  
  laserData2Container( scanArray[0], laserPoints[0] );
  laserData2Container( scanArray[1], laserPoints[1] );
  
  return 0;
}
