#ifndef __OCCUPIED_GRID_MAP_H_
#define __OCCUPIED_GRID_MAP_H_

#include "mapInfo.h"

namespace slam {

template<typename MapBaseType>
class OccupiedGridMap : public MapBaseType
{
public:
	OccupiedGridMap();
	OccupiedGridMap( int sizeX_, int sizeY, float cellLength_ );	
	OccupiedGridMap( const MapInfo &mapInfo );

	~OccupiedGridMap();


};

template<typename MapBaseType>
OccupiedGridMap<MapBaseType>::OccupiedGridMap() : MapBaseType()
{

}

template<typename MapBaseType>
OccupiedGridMap<MapBaseType>::OccupiedGridMap( int sizeX_, int sizeY_, float cellLength_ ) : MapBaseType( sizeX_, sizeY_, cellLength_ )
{

}

template<typename MapBaseType>
OccupiedGridMap<MapBaseType>::OccupiedGridMap( const MapInfo &mapInfo ) : MapBaseType( mapInfo )
{

}

}

#endif
