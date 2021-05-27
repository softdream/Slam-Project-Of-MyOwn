#ifndef __OCCUPIED_GRID_MAP_H_
#define __OCCUPIED_GRID_MAP_H_

#include "gridCell.h"
#include "mapInfo.h"
#include <cmath>

#include "scanContainer.h"

namespace slam {

template<typename MapBaseType>
class OccupiedGridMap : public MapBaseType
{
public:
	OccupiedGridMap();
	OccupiedGridMap( int sizeX_, int sizeY, float cellLength_ );	
	OccupiedGridMap( const MapInfo &mapInfo );

	~OccupiedGridMap();

	Eigen::Vector2f observedPointPoseLaser2World( Eigen::Vector2f &poseInLaser, Eigen::Vector3f &robotPoseInWorld );
	Eigen::Vector2f observedPointPoseWorld2Laser( Eigen::Vector2f &poseInWorld, Eigen::Vector3f &robotPoseInWorld );

	void inverseModel( int x0, int y0, int x1, int y1 );
	void inverseModel( Eigen::Vector2i &p0, Eigen::Vector2i &p1 );
	
	void updateByScan_test( ScanContainer &points, Eigen::Vector3f &robotPoseInWorld );
	

private:
	void bresenHam( int x0, int y0, int x1, int y1 );
	void bresenHam( Eigen::Vector2i &p0, Eigen::Vector2i &p1);

	void bresenhamCellFree( int index );
	void bresenhamCellOccupied( int index );
	void bresenhamCellFree( int mapX, int mapY );
        void bresenhamCellOccupied( int mapX, int mapY );

private:
	int currUpdateIndex;
};

template<typename MapBaseType>
OccupiedGridMap<MapBaseType>::OccupiedGridMap() : MapBaseType(), currUpdateIndex(0)
{

}

template<typename MapBaseType>
OccupiedGridMap<MapBaseType>::OccupiedGridMap( int sizeX_, int sizeY_, float cellLength_ ) : MapBaseType( sizeX_, sizeY_, cellLength_ ), currUpdateIndex( 0 )
{

}

template<typename MapBaseType>
OccupiedGridMap<MapBaseType>::OccupiedGridMap( const MapInfo &mapInfo ) : MapBaseType( mapInfo ), currUpdateIndex(0)
{

}

template<typename MapBaseType>
OccupiedGridMap<MapBaseType>::~OccupiedGridMap()
{

}

template<typename MapBaseType>
void OccupiedGridMap<MapBaseType>::bresenHam( int x0, int y0, int x1, int y1 )
{
	int dx = ::abs( x1 - x0 );
	int dy = ::abs( y1 - y0 );
	
	bool interChange = false;
	
	int e = -dx;// error

	int signX = x1 > x0 ? 1 : ( ( x1 < x0 ) ? -1 : 0 );
	int signY = y1 > y0 ? 1 : ( ( y1 < y0 ) ? -1 : 0 );

	if (dy > dx) {
		int temp = dx;
		dx = dy;
		dy = temp;
		interChange = true;
	}

	int x = x0, y = y0;
	for (int i = 1; i <= dx; i++) { // not include the end point
		//operate( x, y, img1 );
		// add operations for the point 
		bresenhamCellFree( x, y );		
		
		///////////////////////////////
		if (!interChange)
			x += signX;
		else
			y += signY;

		e += 2 * dy;

		if (e >= 0) {
			if (!interChange)
				y += signY;
			else
				x += signX;

			e -= 2 * dx;
		}
	}
}

template<typename MapBaseType>
void OccupiedGridMap<MapBaseType>::bresenHam( Eigen::Vector2i &p0, Eigen::Vector2i &p1 )
{
	int x0 = p0[0];
	int y0 = p0[1];
	int x1 = p1[0];
	int y1 = p1[1];

	bresenHam( x0, y0, x1, y1 );

}

template<typename MapBaseType>
void OccupiedGridMap<MapBaseType>::inverseModel( int x0, int y0, int x1, int y1 )
{
	// 1. set the end point occupied first
	bresenhamCellOccupied( x1, y1 );
	
	// 2. execute the bresenham algorithm, find the points of free, and set them free
	bresenHam( x0, y0, x1, y1 );
}

template<typename MapBaseType>
void OccupiedGridMap<MapBaseType>::inverseModel( Eigen::Vector2i &p0, Eigen::Vector2i &p1 )
{
	int x0 = p0[0];
        int y0 = p0[1];
        int x1 = p1[0];
        int y1 = p1[1];
	
	bresenHam( x0, y0, x1, y1 );
}

template<typename MapBaseType>
void OccupiedGridMap<MapBaseType>::bresenhamCellFree( int index )
{
	GridCell &cell = this->getCell( index );	
	
	if( cell.updateIndex < currUpdateIndex ){
		this->setCellFree( index );
		
		cell.updateIndex = currUpdateIndex; // avoid reUpdate
	}
}

template<typename MapBaseType>
void OccupiedGridMap<MapBaseType>::bresenhamCellOccupied( int index )
{
	GridCell &cell = this->getCell( index );
	
	if( cell.updateIndex < currUpdateIndex ){
                this->setCellOccupied( index );

                cell.updateIndex = currUpdateIndex; // avoid reUpdate
        }

}

template<typename MapBaseType>
void OccupiedGridMap<MapBaseType>::bresenhamCellFree( int mapX, int mapY )
{
        GridCell &cell = this->getCell( mapX, mapY );
	
	if( cell.updateIndex < currUpdateIndex ){
                this->setCellFree( mapX, mapY );

                cell.updateIndex = currUpdateIndex; // avoid reUpdate
        }

}

template<typename MapBaseType>
void OccupiedGridMap<MapBaseType>::bresenhamCellOccupied( int mapX, int mapY )
{
	GridCell &cell = this->getCell( mapX, mapY );

        if( cell.updateIndex < currUpdateIndex ){
                this->setCellOccupied( mapX, mapY );

                cell.updateIndex = currUpdateIndex; // avoid reUpdate
        }

}

template<typename MapBaseType>
Eigen::Vector2f OccupiedGridMap<MapBaseType>::observedPointPoseLaser2World( Eigen::Vector2f &poseInLaser, Eigen::Vector3f &robotPoseInWorld )
{
	//Eigen::Vector2f poseInWorld_temp( ( ::cos( robotPoseInWorld[2] ) * poseInLaser[0] - ::sin( robotPoseInWorld[2] ) * poseInLaser[1] ), ( ::sin( robotPoseInWorld[2] ) * poseInLaser[0] + ::cos( robotPoseInWorld[2] ) * poseInLaser[1] ) );

	//return poseInWorld_temp + robotPoseInWorld.head<2>();	

	Eigen::Matrix2f rotateMat;
	rotateMat << ::cos( robotPoseInWorld[2] ), -(::sin( robotPoseInWorld[2] )),
			::sin( robotPoseInWorld[2] ), ::cos( robotPoseInWorld[2] );

	return rotateMat * poseInLaser + robotPoseInWorld.head<2>();
}

template<typename MapBaseType>
Eigen::Vector2f OccupiedGridMap<MapBaseType>::observedPointPoseWorld2Laser( Eigen::Vector2f &poseInWorld, Eigen::Vector3f &robotPoseInWorld )
{
	Eigen::Matrix2f rotateMat;
        rotateMat << ::cos( robotPoseInWorld[2] ), -(::sin( robotPoseInWorld[2] )),
                        ::sin( robotPoseInWorld[2] ), ::cos( robotPoseInWorld[2] );

	return rotateMat.inverse() * poseInWorld - robotPoseInWorld.head<2>();
}


template<typename MapBaseType>
void OccupiedGridMap<MapBaseType>::updateByScan_test( ScanContainer &points, Eigen::Vector3f &robotPoseInWorld )
{
	currUpdateIndex ++;
	
	
}


}



#endif
