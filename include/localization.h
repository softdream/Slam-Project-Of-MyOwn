#ifndef __LOCALIZATION_H_
#define __LOCALIZATION_H_

#include "scanMatch.h"
#include "dataType.h"

namespace slam{

class Localization
{
public:
	Localization();
	~Localization();

	void setInitialPose(const Eigen::Vector3f &pose);
	
	bool update( const ScanContainer &scanContainer, bool mapWithoutMatching = false );
	
	bool update( const OccupiedMap &occupiedMap, 
		     Eigen::Vector3f &pose, 
		     const ScanContainer &scanContainer, 
		     bool mapWithoutMatching = false );

	const Eigen::Vector3f getLastScanMatchPose() const;

	const Eigen::Matrix3f getCovarianceMatrix() const;
	
	void setOccupiedMap( const OccupiedMap &occupiedMapPtr );

	void setScanMatch( const ScanMatchMethod *scanMatchPtr );

private:
	OccupiedMap occupiedGridMap;
        ScanMatchMethod scanMatch;

	// some important varibles 
	Eigen::Vector3f lastScanMatchPose;
	
	Eigen::Matrix3f covarianceMatrix;
	
	bool isPoseInitialized = false;
};

}

#endif
