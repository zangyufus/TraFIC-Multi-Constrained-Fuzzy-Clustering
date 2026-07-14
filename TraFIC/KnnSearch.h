#pragma once
#include "RegistrationStruct.h"

class CKnnSearch
{
public:
	CKnnSearch();
	~CKnnSearch();



	void findsamePts(std::vector<POINT_STRUCT>& ptsArray_sparse, std::vector<POINT_STRUCT>& ptsArray);

	void run(std::vector<POINT_STRUCT>& ptsArray);
};