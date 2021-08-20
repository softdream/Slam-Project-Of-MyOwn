#ifndef __ODOM_SIMULATION_H_
#define __ODOM_SIMULATION_H_

#include <fstream>
#include <string.h>
#include <sstream>
#include <iomanip>

#include <stdlib.h>
#include <stdint.h>

#include <Eigen/Dense>

namespace slam{

namespace simulation{

class OdomSimulation
{
public:
	OdomSimulation();
	~OdomSimulation();

	bool openSimulationFile( const std::string &inputFile );
        void closeSimulationFile();

	bool readAFrameData( Eigen::Vector3f &odom );

        inline const int filePointPose()
        {
                return input_file.tellg();
        }

        inline const int endOfFile()
        {
                return input_file.eof();
        }

        inline const long getFrameCount() const
        {
                return count;
        }


private:
	std::ifstream input_file;
        long count;

};

}

}

#endif
