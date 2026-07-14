#pragma once
#include "RegistrationStruct.h"
#include "BAMatrix.h"

class COutdata
{
public:
	COutdata();
	~COutdata();



	void ouTransformedPly(char* outfile, CBAMatrix& R, CBAMatrix& T, std::vector<POINT_STRUCT> y_ptsArray);

	void outputPtsRenderedByCur(char* filename, std::vector<POINT_STRUCT> ptsArray);

	void outputPtsRenderedByNorm(char* filename, std::vector<POINT_STRUCT>& pointArray);
};

