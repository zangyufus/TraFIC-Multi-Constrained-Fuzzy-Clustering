#pragma once // 防止头文件被重复包含
#include <vector>
#include <string>
#include <algorithm>
#include <cstring>


#include <Eigen/Dense> 
#include <pcl/point_types.h>

// =================================================================================
// 1. 全局宏定义 (合并自两个文件)
// =================================================================================
#define ZYFMAX 1.79769e+308
#define ZYFMIN -1.79769e+308
#define ZYFPI  3.1415926535897

#define KNUM 90
#define POINT_SPAN 0.1
#define RADIUS (20.0 * POINT_SPAN)
#define GRIDSIZE (1.0 * POINT_SPAN)
#define PROPAGA_REGION 10


// ... (FEATURE_VECTOR, UNIFORM_GRID, UVPARAMETERS, TRANSFORMATION, CategoryPoint 的定义和之前一样) ...
struct FEATURE_VECTOR { double aph, betta, kappa, h_dis, l_dis, d_dis, curvature, curvedness; };
struct UNIFORM_GRID { std::vector<struct POINT_STRUCT> filled_pts_array; std::vector<double> project_dis_array; int isEmpty; int iscurrentPt; UNIFORM_GRID() : isEmpty(1), iscurrentPt(0) {} };
struct UVPARAMETERS { double u, v; std::vector<double> directedLine; double dist; double HorizontalDist, VerticalDist; double project_x, project_y, project_z; double projection_dis; UVPARAMETERS() : u(0.0), v(0.0), dist(0.0), HorizontalDist(0.0), VerticalDist(0.0), project_x(0.0), project_y(0.0), project_z(0.0), projection_dis(0.0) {} };
struct TRANSFORMATION { float R[3][3]; float t[3]; };
//struct CategoryPoint { double x, y, z; int category_id; };


enum class PointCategory 
{
	XUANLIANXIAN = 0,    // 悬链线
	DAOCHUANG = 1,       // 道床
	TIEGUI = 2,          // 铁轨
	ZHIBEI = 3,          // 植被
	DIANXIANGAN = 4,     //电线杆
	ZHENMU = 5,          //枕木
	DIMIAN = 6,          // 地面
	WEILAN = 7,          //围栏
	JIANZHUWU = 8,       //建筑物
	UNKNOWN = -1        // 未知类别
};


struct POINT_STRUCT 
{
	
	double p_x, p_y, p_z;
	std::vector<long int> knnPts_array;
	double normVector[3];
	int ownIndex;

	std::vector<double> centerPoint;    // 存储邻域点的质心坐标

	double u, v;
	double unitVector_u[3];
	double unitVector_v[3];
	double k1Cur, k2Cur;
	double w_interpolation;
	double projection_dis;

	
	double covM[8][8];                  // 描述子

	
	double curvature;
	double meanCurvature, gaussCurvature;
	int isCluster;
	int classNo;
	double cos_value;
	int IsDiscarded;
	int flag;
	bool is_named;
	double weight;

	
	int category = -1; // 存储点的类别标签，默认为-1 (表示未知或未分类)

	
	double E, F, G, L, M, N;
	double similarity;
	int isOutlier;
	double unitVectorOfCurve[3];

	POINT_STRUCT() 
	{
		
		p_x = p_y = p_z = 0.0;
		ownIndex = -1;
		u = v = 0.0;
		meanCurvature = gaussCurvature = 0.0;
		k1Cur = k2Cur = 0.0;
		curvature = 0.0;
		w_interpolation = 0.0; 
		similarity = -1.0;
		isOutlier = 0;
		isCluster = 0;
		classNo = 0;
		cos_value = 0.0;
		IsDiscarded = 0;
		flag = -1;
		projection_dis = 0.0;
		E = F = G = L = M = N = 0.0;
		is_named = false;
		weight = 1.0;

		
		category = -1;

		std::fill_n(normVector, 3, 0.0);
		std::fill_n(unitVector_u, 3, 0.0);
		std::fill_n(unitVector_v, 3, 0.0);
		std::fill_n(unitVectorOfCurve, 3, 0.0);

		// 初始化 covM 数组为0
		for (int i = 0; i < 8; i++) {
			for (int j = 0; j < 8; j++) {
				covM[i][j] = 0.0;
			}
		}
	}

	// 辅助函数：方便地转换为PCL点类型
	pcl::PointXYZ toPclPointXYZ() const {
		return pcl::PointXYZ(static_cast<float>(p_x), static_cast<float>(p_y), static_cast<float>(p_z));
	}
};