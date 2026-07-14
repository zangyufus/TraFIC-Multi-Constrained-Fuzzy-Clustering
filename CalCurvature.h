#pragma once
#include "RegistrationStruct.h"
#include "BAMatrix.h"

class CCalCurvature
{
public:
	CCalCurvature(void);
public:
	~CCalCurvature(void);


protected:
	//Project points
	void calLineParameters(POINT_STRUCT& currentPoint, double& a,double& b, double& c, double& d);
	void calProjectPoints(std::vector<POINT_STRUCT>& kPoints_array, POINT_STRUCT& currentpoint, double a,double b, double c, double d, std::vector<POINT_STRUCT>& projectPoints_array);
	
	//UV parameter
	void calDirectedLineAndDist(std::vector<POINT_STRUCT>& projectPoints_array, std::vector<UVPARAMETERS>& UV_vector);
	void searchMaxMinDistAndCalU(std::vector<UVPARAMETERS>& UV_vector, POINT_STRUCT& currentPoint);

	//caculate curvature
	void fillMZmatrixAndCompute(std::vector<UVPARAMETERS> UV_vector, std::vector<POINT_STRUCT> kPoints_array);
	void calCurvature(POINT_STRUCT& currentPoint);


public:
	void run(std::vector<POINT_STRUCT>& pointArray);


protected:
	CBAMatrix m_Mmatrix;
	CBAMatrix m_Zmatrix;
	CBAMatrix m_Qmatrix;
	CBAMatrix m_onepoint_Mmatrix;

	CBAMatrix ru, rv, ruu, ruv, rvv;
};
