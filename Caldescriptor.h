#pragma once
#include "RegistrationStruct.h"

class CCaldescriptor
{
public:
	CCaldescriptor();
	~CCaldescriptor();


	void calwiforEachpoint(POINT_STRUCT& currentPt, std::vector<POINT_STRUCT>& neighbors);

	void calabhAndFormMatrix(POINT_STRUCT& currentPt, std::vector<POINT_STRUCT>& neighbors);
};

