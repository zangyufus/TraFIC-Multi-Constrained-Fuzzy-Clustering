#include "StdAfx.h"
#include "CalEigen.h"


CCalEigen::CCalEigen(void)
{
}

CCalEigen::~CCalEigen(void)
{
}


void CCalEigen::formMatrix(std::vector<POINT_STRUCT>& kPoints_array, POINT_STRUCT& point, double *a)
{
	double totalx = 0.0;
	double totaly = 0.0;
	double totalz = 0.0;
	for(int i = 0; i < kPoints_array.size(); i++)
	{
		totalx += kPoints_array[i].p_x;
		totaly += kPoints_array[i].p_y;
		totalz += kPoints_array[i].p_z;
	}
	point.centerPoint.push_back(totalx / kPoints_array.size());
	point.centerPoint.push_back(totaly / kPoints_array.size());
	point.centerPoint.push_back(totalz / kPoints_array.size());

	for(int i = 0; i < kPoints_array.size(); i++)
	{
		double dvalueX = kPoints_array[i].p_x - point.centerPoint[0];
		double dvalueY = kPoints_array[i].p_y - point.centerPoint[1];
		double dvalueZ = kPoints_array[i].p_z - point.centerPoint[2];

		a[0] += dvalueX*dvalueX;
		a[1] += dvalueX*dvalueY;
		a[2] += dvalueX*dvalueZ;

		a[3] += dvalueX*dvalueY;
		a[4] += dvalueY*dvalueY;
		a[5] += dvalueZ*dvalueY;

		a[6] += dvalueX*dvalueZ;
		a[7] += dvalueY*dvalueZ;
		a[8] += dvalueZ*dvalueZ;
	}
}

//a 存放n阶实对称矩阵，返回时对角线存放n个特征值
//n 矩阵阶数
//V n阶矩阵，第j列对应第j个特征值的特征向量
//eps 控制精度
//jt 最大迭代次数
int CCalEigen::jacobi(double *a, int n, double *v, double eps, int jt)
{
	int i, j, p, q, u, w, t, s, l;
    double fm, cn, sn, omega, x, y, d;
    l = 1;
	for (i = 0; i <= n - 1; i++)
	{
		v[i * n + i] = 1.0;
		for (j = 0; j <= n - 1; j++)
		{
			if (i != j) 
				v[i * n + j] = 0.0;
		}
	}

    while (1)
	{ 
		fm = 0.0;
		for(i = 1; i <= n-1; i++)
		{
			for(j = 0; j <= i-1; j++)
			{ 
				d = fabs(a[i * n + j]);
				if((i != j) && (d > fm))
				{ 
					fm = d; p = i;q = j;
				}
			}
		}

        if (fm < eps) 
			return(1);
        if (l > jt) 
			return(-1);

        l = l + 1;
        u = p * n + q; 
		w = p * n + p; 
		t = q * n + p; 
		s = q * n + q;
        x = -a[u]; 
		y = (a[s] - a[w]) / 2.0;
        omega = x / sqrt(x * x + y * y);

        if(y < 0.0) 
			omega = -omega;
        sn = 1.0 + sqrt(1.0 - omega * omega);
        sn = omega / sqrt(2.0 * sn);
        cn = sqrt(1.0 - sn * sn);
        fm = a[w];
        a[w] = fm * cn * cn + a[s] * sn * sn + a[u] * omega;
        a[s] = fm * sn * sn + a[s] * cn * cn - a[u] * omega;
        a[u] = 0.0; a[t] = 0.0;
		for(j = 0; j <= n-1; j++)
		{
			if((j != p) && (j != q))
			{
				u = p * n + j; w = q * n + j;
				fm = a[u];
				a[u] = fm * cn + a[w] * sn;
				a[w] = -fm * sn + a[w] * cn;
			}
		}

		for(i = 0; i <= n-1; i++)
		{		
			if ((i != p) && (i != q))
			{ 
				u = i * n + p; w = i * n + q;
				fm = a[u];
				a[u] = fm * cn + a[w] * sn;
				a[w] = -fm * sn + a[w] * cn;
			}
		}

		for(i = 0; i <= n-1; i++)
		{ 
			u = i * n + p; w = i * n + q;
			fm = v[u];
			v[u] = fm * cn + v[w] * sn;
			v[w] = -fm * sn + v[w] * cn;
		}
      }

    return(1);
}

void CCalEigen::calEps(double& eps, double *a)
{
	if(abs(a[0]) < eps*10)
		eps = abs(a[0]) / 10000;

	if(abs(a[4]) < eps*10)
		eps = abs(a[4]) / 10000;

	if(abs(a[8]) < eps*10)
		eps = abs(a[8]) / 10000;
}



void CCalEigen::run(std::vector<POINT_STRUCT>& pointArray)
{
	for(int i = 0; i < pointArray.size(); i++)
	{
		POINT_STRUCT& currentpoint = pointArray[i];
		std::vector<POINT_STRUCT> kPoints_array;
		kPoints_array.push_back(currentpoint);
		for(int j = 0; j < currentpoint.knnPts_array.size(); j++)
		{
			POINT_STRUCT& neighbor_point = pointArray[currentpoint.knnPts_array[j]];
			kPoints_array.push_back(neighbor_point);
		}

		double a[3][3] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ,0.0};
		formMatrix(kPoints_array, currentpoint,(double*)a);
		

		double v[3][3];
		double eps = 1.0e-9;
		calEps(eps, (double*)a);
		int iterNum = 100;
		jacobi((double*)a, 3, (double*)v, eps, iterNum);

		

		double tempBuffer = a[0][0];
		currentpoint.normVector[0] = v[0][0];
		currentpoint.normVector[1] = v[1][0];
		currentpoint.normVector[2] = v[2][0];
		if(tempBuffer > a[1][1])
		{
			tempBuffer = a[1][1];
			currentpoint.normVector[0] = v[0][1];
			currentpoint.normVector[1] = v[1][1];
			currentpoint.normVector[2] = v[2][1];
		}
		if(tempBuffer > a[2][2])
		{
			tempBuffer = a[2][2];
			currentpoint.normVector[0] = v[0][2];
			currentpoint.normVector[1] = v[1][2];
			currentpoint.normVector[2] = v[2][2];
		}
	}

}

