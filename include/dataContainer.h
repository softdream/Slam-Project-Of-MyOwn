#ifndef __DATA_CONTAINER_H_
#define __DATA_CONTAINER_H_

#include <vector>
#include <cmath>

#include <opencv2/opencv.hpp>

namespace slam {

template<typename DataType>
class DataContainer
{
public:
	DataContainer();
	~DataContainer();

	DataContainer( const DataContainer &rhs );
	const DataContainer& operator=( const DataContainer &rhs );

	void addData( const DataType &data );
	void clear();
	
	const DataType& getIndexData( int index ) const;

	template<typename LaserData>
	void pointTransform2LaserCoords( const LaserData &scan );	
	
	int getSize() const;

	void displayAFrameScan( int num, const float scale ) const;
	
private:
	std::vector<DataType> dataVec;
	
};

template<typename DataType>
DataContainer<DataType>::DataContainer()
{
	
}

template<typename DataType>
DataContainer<DataType>::~DataContainer()
{

}

template<typename DataType>
DataContainer<DataType>::DataContainer( const DataContainer &rhs ) : dataVec( rhs.dataVec )
{

}

template<typename DataType>
const DataContainer<DataType>& DataContainer<DataType>::operator=( const DataContainer &rhs )
{
	if( &rhs == this ){
		return *this;
	}

	this->dataVec = rhs.dataVec;

	return *this;
}


template<typename DataType>
int DataContainer<DataType>::getSize() const
{
	return dataVec.size();
}


template<typename DataType>
void DataContainer<DataType>::addData( const DataType &data )
{
	dataVec.push_back( data );
}

template<typename DataType>
void DataContainer<DataType>::clear()
{
	dataVec.clear();
}


template<typename DataType>
const DataType& DataContainer<DataType>::getIndexData( int index ) const
{
        return dataVec[index];
}

template<typename DataType>
template<typename LaserData>
void DataContainer<DataType>::pointTransform2LaserCoords( const LaserData &scan )
{
	int size = scan.size();
	
	float angle = scan.angle_min;
	
	this->clear();

	float range_max = scan.range_max - 0.1f;

	for( int i = 0; i < size; i ++ ){
		float dist = scan.ranges[i];
	
		if( ( dist > scan.range_min ) && ( dist < range_max ) ){
			DataType pointInLaserCoords( ::cos(angle) * dist, ::sin(angle) * dist );
			this->addData( pointInLaserCoords );
		}

		angle += scan.angle_increment;
	}
}

template<typename DataType>
void DataContainer<DataType>::displayAFrameScan( int num, const float scale ) const
{
	cv::Mat image = cv::Mat::zeros( 600, 600, CV_8UC3 );
	cv::Point2d center( 300, 300 );
	cv::circle(image, center, 1, cv::Scalar(0, 0, 255), 1);	

	cv::line( image, cv::Point( 300, 0 ), cv::Point( 300, 600 ), cv::Scalar( 67, 128, 94 ), 1 );
        cv::line( image, cv::Point( 0, 300 ), cv::Point( 600, 300 ), cv::Scalar( 67, 128, 94 ), 1 );

	for( auto it : dataVec ){
		cv::Point2d point( it(0) * scale + 300, it(1) * scale + 300 );
		cv::circle(image, point, 1, cv::Scalar(0, 0, 255), 1);
	} 

	cv::imshow( "scan" + std::to_string( num ), image );
}


}

#endif
