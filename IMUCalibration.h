#ifndef __IMU_CALIBRATION_H_
#define __IMU_CALIBRATION_H_

#include <Eigen/Dense>
#include <vector>

#include "dataType.h"

namespace slam{

namespace sensor{

class IMUCalibration
{
public:
	IMUCalibration();
	~IMUCalibration();

	void addDataOfAccelerometer( const IMU &imu );
	void addDataOfGyrometer( const IMU &imu );

	bool calibrateAccelerometer();
	bool calibrateGyrometer();

	Eigen::RowVector3f getAccelBias() const;
	Eigen::Vector3f getGyroBias() const;

	float getAccelBiasX() const;
	float getAccelBiasY() const;
	float getAccelBiasZ() const;

	float getGyroBiasX() const;
	float getGyroBiasY() const;
	float getGyroBiasZ() const;

private:
	template<typename T>
	T square(const T &num );

	bool caculateParametersA();

private:
	std::vector<Eigen::Matrix<float, 9, 1>> accelArray;
	std::vector<Eigen::Vector3f> gyroArray;

	Eigen::Matrix<float, 9, 1> parameters;
	Eigen::RowVector3f ellipsoidCenter;

	Eigen::Vector3f gyroBias;
};

}

}

#endif
