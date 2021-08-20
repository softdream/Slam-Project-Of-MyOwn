#include "odomSimulation.h"
#include <iostream>

namespace slam{

namespace simulation{

OdomSimulation::OdomSimulation()
{

}

OdomSimulation::~OdomSimulation()
{

}

bool OdomSimulation::openSimulationFile( const std::string &inputFile )
{
	input_file.open( inputFile.c_str(), std::ifstream::in );

        if( !input_file.is_open() ){
                std::cout<<"Failed to open the simulation file ..."<<std::endl;
                return false;
        }

        std::cout<<"............Open the Simulation File ............."<<std::endl;
}

void OdomSimulation::closeSimulationFile()
{
        return input_file.close();
}

bool OdomSimulation::readAFrameData( Eigen::Vector3f &odom )
{

        std::string line;

        std::getline(input_file, line);
        {
                //std::cout << line << std::endl;
                std::istringstream iss(line);
                std::string tag;
                iss >> tag;
                std::string num;

                if (tag.compare("odom") == 0) {

                        for (int i = 0; i < 3; i++) {
                                iss >> num;
                                //std::cout << num << "\t";
                                odom[i] = std::stof( num );
                        }
                        count++;
                }
        }
}


}

}
