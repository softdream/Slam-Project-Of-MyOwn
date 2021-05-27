#ifndef __DATA_CONTAINER_H_
#define __DATA_CONTAINER_H_

#include <vector>
#include <cmath>

namespace slam {

template<typename DataType>
class DataContainer
{
public:
	DataContainer();
	~DataContainer();

	void addData( const DataType &data );
	void clear();
	
	const DataType& getIndexData( int index ) const;

	template<typename LaserData>
	void pointTransform2LaserCoords( const LaserData &scan );	

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


}

#endif
