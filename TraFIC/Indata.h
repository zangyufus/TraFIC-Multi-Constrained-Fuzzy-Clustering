#pragma once
#include "RegistrationStruct.h"

class CIndata
{
public:
	CIndata();
	~CIndata();


	void run(const char* infile, std::vector<POINT_STRUCT>& ptsArray);

	void readsamplingPts(char* infile, std::vector<POINT_STRUCT>& ptsArray);
};

