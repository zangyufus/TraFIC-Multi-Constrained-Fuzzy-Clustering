#include "StdAfx.h"
#include "CalCurvature.h"



CCalCurvature::CCalCurvature(void)
{
}

CCalCurvature::~CCalCurvature(void)
{
}


void CCalCurvature::calLineParameters(POINT_STRUCT& currentPoint, double& a,double& b, double& c, double& d)
{
	a = currentPoint.normVector[0];
	b = currentPoint.normVector[1];
	c = currentPoint.normVector[2];

	d = -a * currentPoint.centerPoint[0] - b * currentPoint.centerPoint[1] - c * currentPoint.centerPoint[2];
}

void CCalCurvature::calProjectPoints(std::vector<POINT_STRUCT>& kPoints_array, POINT_STRUCT& currentpoint, double a,double b, double c, double d, std::vector<POINT_STRUCT>& projectPoints_array)
{
	for(int i = 0; i < kPoints_array.size(); i++)
	{
		double t = 0.0;
		t = -(a* kPoints_array[i].p_x + b* kPoints_array[i].p_y + c* kPoints_array[i].p_z + d) / (a* a + b* b + c* c);

		POINT_STRUCT projectpoint;
		projectpoint.p_x = currentpoint.normVector[0] * t + kPoints_array[i].p_x;
		projectpoint.p_y = currentpoint.normVector[1] * t + kPoints_array[i].p_y;
		projectpoint.p_z = currentpoint.normVector[2] * t + kPoints_array[i].p_z;
		projectPoints_array.push_back(projectpoint);
	}
}


void CCalCurvature::calDirectedLineAndDist(std::vector<POINT_STRUCT>& projectPoints_array, std::vector<UVPARAMETERS>& UV_vector)
{
	UVPARAMETERS uvparameters;
	uvparameters.directedLine.push_back(0);
	uvparameters.directedLine.push_back(0);
	uvparameters.directedLine.push_back(0);
	uvparameters.dist = 0.0;
	UV_vector.push_back(uvparameters);

	POINT_STRUCT& curentPoint = projectPoints_array[0];
	for(int i = 1; i < projectPoints_array.size(); i++)
	{
		UVPARAMETERS uvparameters;
		uvparameters.directedLine.push_back( projectPoints_array[i].p_x - curentPoint.p_x);
		uvparameters.directedLine.push_back( projectPoints_array[i].p_y - curentPoint.p_y);
		uvparameters.directedLine.push_back( projectPoints_array[i].p_z - curentPoint.p_z);
		uvparameters.dist = sqrt(uvparameters.directedLine[0] * uvparameters.directedLine[0] + uvparameters.directedLine[1] * uvparameters.directedLine[1] + 
			                 uvparameters.directedLine[2] * uvparameters.directedLine[2]);
		UV_vector.push_back(uvparameters);
	}
}

void CCalCurvature::searchMaxMinDistAndCalU(std::vector<UVPARAMETERS>& UV_vector, POINT_STRUCT& currentPoint)
{
	int maxIndex = 1;//2019.2.19 100个邻域点死机的现象解决
	double maxDist = UV_vector[1].dist;
	for(int i = 1; i < UV_vector.size(); i++)
	{
		if(UV_vector[i].dist > maxDist)
		{
			maxDist = UV_vector[i].dist;
			maxIndex = i;
		}		
	}

	double norm_u = sqrt(UV_vector[maxIndex].directedLine[0]* UV_vector[maxIndex].directedLine[0] + UV_vector[maxIndex].directedLine[1]* UV_vector[maxIndex].directedLine[1] + 
						 UV_vector[maxIndex].directedLine[2]* UV_vector[maxIndex].directedLine[2]);
	currentPoint.unitVector_u[0] = UV_vector[maxIndex].directedLine[0] / norm_u;
	currentPoint.unitVector_u[1] = UV_vector[maxIndex].directedLine[1] / norm_u;
	currentPoint.unitVector_u[2] = UV_vector[maxIndex].directedLine[2] / norm_u;

	//拿平面法向量差乘unitVector_u
	currentPoint.unitVector_v[0] = currentPoint.normVector[1]*currentPoint.unitVector_u[2] - currentPoint.normVector[2]*currentPoint.unitVector_u[1];
	currentPoint.unitVector_v[1] = currentPoint.normVector[2]*currentPoint.unitVector_u[0] - currentPoint.normVector[0]*currentPoint.unitVector_u[2];
	currentPoint.unitVector_v[2] = currentPoint.normVector[0]*currentPoint.unitVector_u[1] - currentPoint.normVector[1]*currentPoint.unitVector_u[0];
	double norm_v = sqrt(currentPoint.unitVector_v[0]*currentPoint.unitVector_v[0] + currentPoint.unitVector_v[1]*currentPoint.unitVector_v[1] + currentPoint.unitVector_v[2]* currentPoint.unitVector_v[2]);
	currentPoint.unitVector_v[0] = currentPoint.unitVector_v[0] / norm_v;
	currentPoint.unitVector_v[1] = currentPoint.unitVector_v[1] / norm_v;
	currentPoint.unitVector_v[2] = currentPoint.unitVector_v[2] / norm_v;

	double maxHorizontalDist, minHorizontalDist;
	double maxVerticalDist, minVerticalDist;
	maxHorizontalDist = maxVerticalDist = ZYFMIN;
	minHorizontalDist = minVerticalDist = ZYFMAX;
	for(int i = 1; i < UV_vector.size(); i++)
	{
		UV_vector[i].HorizontalDist = UV_vector[i].directedLine[0] * currentPoint.unitVector_u[0] + UV_vector[i].directedLine[1] * currentPoint.unitVector_u[1] + UV_vector[i].directedLine[2] * currentPoint.unitVector_u[2];
		//UV_vector[i].VerticalDist = sqrt(UV_vector[i].dist * UV_vector[i].dist - UV_vector[i].HorizontalDist * UV_vector[i].HorizontalDist);//没负号了！！！！！！！！！！！！！！
		UV_vector[i].VerticalDist = UV_vector[i].directedLine[0] * currentPoint.unitVector_v[0] + UV_vector[i].directedLine[1] * currentPoint.unitVector_v[1] + UV_vector[i].directedLine[2] * currentPoint.unitVector_v[2];		

		if(UV_vector[i].HorizontalDist <= minHorizontalDist)
		{
			minHorizontalDist = UV_vector[i].HorizontalDist;
		}

		if(UV_vector[i].HorizontalDist >= maxHorizontalDist)
		{
			maxHorizontalDist = UV_vector[i].HorizontalDist;
		}	

		if(UV_vector[i].VerticalDist <= minVerticalDist)
		{
			minVerticalDist = UV_vector[i].VerticalDist;
		}

		if(UV_vector[i].VerticalDist >= maxVerticalDist)
		{
			maxVerticalDist = UV_vector[i].VerticalDist;
		}
	}

	if(minHorizontalDist == maxHorizontalDist || minVerticalDist == maxVerticalDist)
		printf("UV参数化不正确,请检查...\n");


	for(int i = 0; i < UV_vector.size(); i++)
	{
		UV_vector[i].u = (UV_vector[i].HorizontalDist - minHorizontalDist) / (maxHorizontalDist - minHorizontalDist);
		UV_vector[i].v = (UV_vector[i].VerticalDist - minVerticalDist) / (maxVerticalDist - minVerticalDist);
	}
	currentPoint.u = UV_vector[0].u;
	currentPoint.v = UV_vector[0].v;
}



//由UV组建参数矩阵Q
//UV_vector与kPoints_array序号一致
void CCalCurvature::fillMZmatrixAndCompute(std::vector<UVPARAMETERS> UV_vector, std::vector<POINT_STRUCT> kPoints_array)
{
	for(int i = 0; i < UV_vector.size(); i++)
	{
		POINT_STRUCT point = kPoints_array[i];
		UVPARAMETERS uvparameters = UV_vector[i];

		m_Mmatrix.setElement(i, 0, 1);
		m_Mmatrix.setElement(i, 1, uvparameters.v);
		m_Mmatrix.setElement(i, 2, uvparameters.v* uvparameters.v);

		m_Mmatrix.setElement(i, 3, uvparameters.u);
		m_Mmatrix.setElement(i, 4, uvparameters.u* uvparameters.v);
		m_Mmatrix.setElement(i, 5, uvparameters.u* uvparameters.v* uvparameters.v);

		m_Mmatrix.setElement(i, 6, uvparameters.u* uvparameters.u);
		m_Mmatrix.setElement(i, 7, uvparameters.u* uvparameters.u* uvparameters.v);
		m_Mmatrix.setElement(i, 8, uvparameters.u* uvparameters.u * uvparameters.v* uvparameters.v);

		m_Zmatrix.setElement(i, 0, point.p_x);
		m_Zmatrix.setElement(i, 1, point.p_y);
		m_Zmatrix.setElement(i, 2, point.p_z);
	}

	//C = A'*B
	CBAMatrix tempMatrix1(9, 9);
	CBAMatrix::TransposeMultiply(m_Mmatrix, m_Mmatrix, tempMatrix1);
	CBAMatrix::Invert(tempMatrix1);

	CBAMatrix tempMatrix2(9, 3);
	CBAMatrix::TransposeMultiply(m_Mmatrix, m_Zmatrix, tempMatrix2);
	
	//C = A*B
	CBAMatrix::Multiply(tempMatrix1, tempMatrix2, m_Qmatrix);
}

//计算该曲面在当前点处的各种曲率
void CCalCurvature::calCurvature(POINT_STRUCT& currentPoint)
{
	m_onepoint_Mmatrix.setElement(0, 0, 0);
	m_onepoint_Mmatrix.setElement(0, 1, 0);
	m_onepoint_Mmatrix.setElement(0, 2, 0);

	m_onepoint_Mmatrix.setElement(0, 3, 1);
	m_onepoint_Mmatrix.setElement(0, 4, currentPoint.v);
	m_onepoint_Mmatrix.setElement(0, 5, currentPoint.v* currentPoint.v);

	m_onepoint_Mmatrix.setElement(0, 6, 2*currentPoint.u);
	m_onepoint_Mmatrix.setElement(0, 7, 2*currentPoint.u*currentPoint.v);
	m_onepoint_Mmatrix.setElement(0, 8, 2*currentPoint.u*currentPoint.v*currentPoint.v);
	//C = A*B
	CBAMatrix::Multiply(m_onepoint_Mmatrix, m_Qmatrix, ru);

	//ruu
	m_onepoint_Mmatrix.setElement(0, 0, 0);
	m_onepoint_Mmatrix.setElement(0, 1, 0);
	m_onepoint_Mmatrix.setElement(0, 2, 0);

	m_onepoint_Mmatrix.setElement(0, 3, 0);
	m_onepoint_Mmatrix.setElement(0, 4, 0);
	m_onepoint_Mmatrix.setElement(0, 5, 0);

	m_onepoint_Mmatrix.setElement(0, 6, 2);
	m_onepoint_Mmatrix.setElement(0, 7, 2*currentPoint.v);
	m_onepoint_Mmatrix.setElement(0, 8, 2*currentPoint.v*currentPoint.v);
	//C = A*B
	CBAMatrix::Multiply(m_onepoint_Mmatrix, m_Qmatrix, ruu);

	//ruv
	m_onepoint_Mmatrix.setElement(0, 0, 0);
	m_onepoint_Mmatrix.setElement(0, 1, 0);
	m_onepoint_Mmatrix.setElement(0, 2, 0);

	m_onepoint_Mmatrix.setElement(0, 3, 0);
	m_onepoint_Mmatrix.setElement(0, 4, 1);
	m_onepoint_Mmatrix.setElement(0, 5, 2*currentPoint.v);

	m_onepoint_Mmatrix.setElement(0, 6, 0);
	m_onepoint_Mmatrix.setElement(0, 7, 2*currentPoint.u);
	m_onepoint_Mmatrix.setElement(0, 8, 4*currentPoint.u*currentPoint.v);
	//C = A*B
	CBAMatrix::Multiply(m_onepoint_Mmatrix, m_Qmatrix, ruv);

	//rv
	m_onepoint_Mmatrix.setElement(0, 0, 0);
	m_onepoint_Mmatrix.setElement(0, 1, 1);
	m_onepoint_Mmatrix.setElement(0, 2, 2*currentPoint.v);

	m_onepoint_Mmatrix.setElement(0, 3, 0);
	m_onepoint_Mmatrix.setElement(0, 4, currentPoint.u);
	m_onepoint_Mmatrix.setElement(0, 5, 2*currentPoint.u*currentPoint.v);

	m_onepoint_Mmatrix.setElement(0, 6, 0);
	m_onepoint_Mmatrix.setElement(0, 7, currentPoint.u*currentPoint.u);
	m_onepoint_Mmatrix.setElement(0, 8, 2*currentPoint.u*currentPoint.u*currentPoint.v);
	//C = A*B
	CBAMatrix::Multiply(m_onepoint_Mmatrix, m_Qmatrix, rv);

	//rvv
	m_onepoint_Mmatrix.setElement(0, 0, 0);
	m_onepoint_Mmatrix.setElement(0, 1, 0);
	m_onepoint_Mmatrix.setElement(0, 2, 2);

	m_onepoint_Mmatrix.setElement(0, 3, 0);
	m_onepoint_Mmatrix.setElement(0, 4, 0);
	m_onepoint_Mmatrix.setElement(0, 5, 2*currentPoint.u);

	m_onepoint_Mmatrix.setElement(0, 6, 0);
	m_onepoint_Mmatrix.setElement(0, 7, 0);
	m_onepoint_Mmatrix.setElement(0, 8, 2*currentPoint.u*currentPoint.u);
	//C = A*B
	CBAMatrix::Multiply(m_onepoint_Mmatrix, m_Qmatrix, rvv);

	//计算曲面的单位法矢，ru、rv差乘
	double unitVectorOfCurve[3];
	unitVectorOfCurve[0] = ru(0, 1)*rv(0, 2) - ru(0, 2)*rv(0, 1);
	unitVectorOfCurve[1] = ru(0, 2)*rv(0, 0) - ru(0, 0)*rv(0, 2);
	unitVectorOfCurve[2] = ru(0, 0)*rv(0, 1) - ru(0, 1)*rv(0, 0);
	double norm = sqrt(unitVectorOfCurve[0]*unitVectorOfCurve[0] + unitVectorOfCurve[1]*unitVectorOfCurve[1] + unitVectorOfCurve[2]*unitVectorOfCurve[2]);
	currentPoint.unitVectorOfCurve[0] = unitVectorOfCurve[0] / norm;
	currentPoint.unitVectorOfCurve[1] = unitVectorOfCurve[1] / norm;
	currentPoint.unitVectorOfCurve[2] = unitVectorOfCurve[2] / norm;

	currentPoint.E = ru(0, 0)*ru(0, 0) + ru(0, 1)*ru(0, 1) + ru(0, 2)*ru(0, 2);
	currentPoint.F = ru(0, 0)*rv(0, 0) + ru(0, 1)*rv(0, 1) + ru(0, 2)*rv(0, 2);
	currentPoint.G = rv(0, 0)*rv(0, 0) + rv(0, 1)*rv(0, 1) + rv(0, 2)*rv(0, 2);

	currentPoint.L = currentPoint.unitVectorOfCurve[0]* ruu(0, 0) + currentPoint.unitVectorOfCurve[1]* ruu(0, 1) + currentPoint.unitVectorOfCurve[2]* ruu(0, 2);
	currentPoint.M = currentPoint.unitVectorOfCurve[0]* ruv(0, 0) + currentPoint.unitVectorOfCurve[1]* ruv(0, 1) + currentPoint.unitVectorOfCurve[2]* ruv(0, 2);
	currentPoint.N = currentPoint.unitVectorOfCurve[0]* rvv(0, 0) + currentPoint.unitVectorOfCurve[1]* rvv(0, 1) + currentPoint.unitVectorOfCurve[2]* rvv(0, 2);

	currentPoint.meanCurvature = (currentPoint.E*currentPoint.N - 2*currentPoint.F*currentPoint.M + currentPoint.G*currentPoint.L) / (2*currentPoint.E*currentPoint.G - 2*currentPoint.F*currentPoint.F);
	currentPoint.gaussCurvature = (currentPoint.L*currentPoint.N - currentPoint.M*currentPoint.M) / (currentPoint.E*currentPoint.G - currentPoint.F*currentPoint.F);


	currentPoint.k1Cur = currentPoint.meanCurvature - sqrt(currentPoint.meanCurvature*currentPoint.meanCurvature - currentPoint.gaussCurvature);
	currentPoint.k2Cur = currentPoint.meanCurvature + sqrt(currentPoint.meanCurvature*currentPoint.meanCurvature - currentPoint.gaussCurvature);
}



void CCalCurvature::run(std::vector<POINT_STRUCT>& pointArray)
{
	for(int i = 0; i < pointArray.size(); i++)
	{
		//将各点的K近邻点取出来kPoints_array
		POINT_STRUCT& current_point = pointArray[i];
		//2019.2.18
		//if(current_point.IsDiscarded == 1)
		//	continue;

		std::vector<POINT_STRUCT> kPoints_array;
		kPoints_array.push_back(current_point);
		for(int j = 0; j < current_point.knnPts_array.size(); j++)
		{
			POINT_STRUCT& point2 = pointArray[pointArray[i].knnPts_array[j]];
			kPoints_array.push_back(point2);
		}

		//第一步 投影各点
		double a , b, c, d;
		std::vector<POINT_STRUCT> projectPoints_array;
		calLineParameters(current_point, a, b, c, d);
		calProjectPoints(kPoints_array, current_point, a, b, c, d, projectPoints_array);


		//第二步 各点u、v参数化
		std::vector<UVPARAMETERS> UV_vector;
		//计算其他点与当前点的有向线段及距离
		calDirectedLineAndDist(projectPoints_array, UV_vector);
		//寻找最大距离及最小距离、计算所有点的U
		searchMaxMinDistAndCalU(UV_vector, current_point);


		//第三步 计算参数矩阵Q  求各种曲率
		int pointNum = UV_vector.size();
		m_Mmatrix.alloc(pointNum, 9);
		m_onepoint_Mmatrix.alloc(1, 9);
		m_Zmatrix.alloc(pointNum, 3);
		m_Qmatrix.alloc(9, 3);	
		ru.alloc(1, 3); rv.alloc(1, 3); ruu.alloc(1, 3); ruv.alloc(1, 3); rvv.alloc(1, 3);
		
		//由UV组建参数矩阵Q
		fillMZmatrixAndCompute(UV_vector, kPoints_array);
		
		//计算该曲面在当前点处的各种曲率
		calCurvature(current_point);
		
		
		//清空
		m_Mmatrix.clear();
		m_onepoint_Mmatrix.clear();
		m_Zmatrix.clear();
		m_Qmatrix.clear();
		ru.clear();rv.clear();ruu.clear();ruv.clear();rvv.clear();
	}
}