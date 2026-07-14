#pragma once
#include "RegistrationStruct.h"

class CCalEigen
{
public:
	CCalEigen(void);
public:
	~CCalEigen(void);


public:
	void formMatrix(std::vector<POINT_STRUCT>& kPoints_array, POINT_STRUCT& point, double *a);
	
	int jacobi(double *a,int n,double *v,double eps,int jt);
	
	void calEps(double& eps, double *a);
	


public:
	
	void run(std::vector<POINT_STRUCT>& pointArray);
};
