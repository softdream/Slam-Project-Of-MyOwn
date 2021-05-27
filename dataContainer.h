#ifndef __DATA_CONTAINER_H_
#define __DATA_CONTAINER_H_

#include <vector>

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



}

#endif
