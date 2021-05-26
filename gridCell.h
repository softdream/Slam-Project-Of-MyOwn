#ifndef __GRID_CELL_H_
#define __GRID_CELL_H_

namespace slam{

class GridCell
{
public:
	GridCell();
	~GridCell();

	void setLogOddsValue( float logOddsValue );

	float getLogOddsValue() const;

	bool isOccupied() const ;
	bool isFree() const;
	bool isUnknow() const;	

	void resetGridCell();

public:
	float logOddsValue;

};

}

#endif
