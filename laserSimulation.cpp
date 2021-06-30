#include "laserSimulation.h"

namespace slam{

namespace simulation{

Simulation::Simulation(): count( 0 )
{

}

Simulation::~Simulation()
{

}

bool Simulation::openSimulationFile( const std::string &inputFile )
{
	input_file.open( inputFile.c_str(), std::ifstream::in );
	
	if( !input_file.is_open() ){
		std::cout<<"Failed to open the simulation file ..."<<std::endl;
		return false;
	}
	
	std::cout<<"............Open the Simulation File ............."<<std::endl;

}

void Simulation::closeSimulationFile()
{
	return input_file.close();
}

bool Simulation::readAFrameData( slam::sensor::LaserScan &scan )
{
	memset( &scan, 0, sizeof( scan ) );	

	std::string line;

	std::getline(input_file, line);
	{
		//std::cout << line << std::endl;
                std::istringstream iss(line);
                std::string tag;
                iss >> tag;
                std::string num;
	
                if (tag.compare("laser") == 0) {
		
			for (int i = 0; i < scan.size(); i++) {
                                iss >> num;
                                //std::cout << num << "\t";
                                //iss >> scan[count].range[i];
                                if (!num.compare("inf")) {
                                        scan.ranges[i] = 65536;
                                }
                                else{
                                        scan.ranges[i] = std::stof( num );
                                }
                        }
                        count++;
			
		}
	}
}

}

}
