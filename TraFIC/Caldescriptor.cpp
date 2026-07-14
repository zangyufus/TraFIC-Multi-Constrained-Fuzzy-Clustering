#include "stdafx.h"
#include "Caldescriptor.h"


CCaldescriptor::CCaldescriptor()
{
}

CCaldescriptor::~CCaldescriptor()
{
}


void CCaldescriptor::calwiforEachpoint(POINT_STRUCT& currentPt, std::vector<POINT_STRUCT>& neighbors)
{
	for (int i = 0; i < neighbors.size(); i++)
	{
		double distance = sqrt( (neighbors[i].p_x - currentPt.p_x) * (neighbors[i].p_x - currentPt.p_x) + 
								(neighbors[i].p_y - currentPt.p_y) * (neighbors[i].p_y - currentPt.p_y) +
								(neighbors[i].p_z - currentPt.p_z) * (neighbors[i].p_z - currentPt.p_z) );
		double neweight; neweight = exp(-0.1 * distance);

		neighbors[i].w_interpolation += neweight;
	}
}

void CCaldescriptor::calabhAndFormMatrix(POINT_STRUCT& currentPt, std::vector<POINT_STRUCT>& neighbors)
{
	//step1 calculate the weight
	calwiforEachpoint(currentPt, neighbors);

	//1.for each neighbor, calculate and store its features
	std::vector<FEATURE_VECTOR> feature_vectors;
	for (int j = 0; j < neighbors.size(); j++)
	{
		FEATURE_VECTOR one_feature_vector;   //计算出一个包含8个元素的特征向量 one_feature_vector
		//三个角元素
		//step1 计算法向量与op夹角a
		double op[3];
		op[0] = neighbors[j].p_x - currentPt.p_x;
		op[1] = neighbors[j].p_y - currentPt.p_y;
		op[2] = neighbors[j].p_z - currentPt.p_z;
		double length1 = sqrt( op[0]*op[0] + op[1]*op[1] + op[2]*op[2] );
		
		if (length1 < 1e-9) // 如果两个点重合或非常近
		{ 
			op[0] = 0; op[1] = 0; op[2] = 0; // 可以给一个零向量
		}
		else 
		{
			op[0] = op[0] / length1; op[1] = op[1] / length1; op[2] = op[2] / length1;
		}


		op[0] = op[0] / length1; op[1] = op[1] / length1; op[2] = op[2] / length1;
		double dotpro1 = op[0]*currentPt.normVector[0] + op[1]*currentPt.normVector[1] + op[2]*currentPt.normVector[2];
		
		if (dotpro1 > 1.0) 
		{
			dotpro1 = 1.0;
		}
		if (dotpro1 < -1.0) 
		{
			dotpro1 = -1.0;
		}
		double arc_aph = acos(dotpro1);
		arc_aph = ZYFPI - arc_aph;
		//double angle_a = arc_a * 180.0 / ZYFPI;	 //转成角度
		one_feature_vector.aph = arc_aph;

		//step2 计算法向量夹角b
		double dotpro2 = neighbors[j].normVector[0]*currentPt.normVector[0] + neighbors[j].normVector[1]*currentPt.normVector[1] +
						 neighbors[j].normVector[2]*currentPt.normVector[2];
		if (dotpro2 > 1.0)
		{
			dotpro2 = 1.0;
		}
		if (dotpro2 < -1.0)
		{
			dotpro2 = -1.0;
		}
		double arc_betta = acos(dotpro2);
		one_feature_vector.betta = arc_betta;
		
		//step3 计算po与邻域点法向量的夹角
		double dotpro3 = -neighbors[j].normVector[0] * op[0] - neighbors[j].normVector[1] * op[1] - neighbors[j].normVector[2] * op[2];

		if (dotpro3 > 1.0)
		{
			dotpro3 = 1.0;
		}
		if (dotpro3 < -1.0)
		{
			dotpro3 = -1.0;
		}
		double arc_kappa = acos(dotpro3);
		one_feature_vector.kappa = arc_kappa;

		//三个线元素
		//step4 计算距离切平面的距离h
		double o2p[3];
		o2p[0] = neighbors[j].p_x - currentPt.p_x;
		o2p[1] = neighbors[j].p_y - currentPt.p_y;
		o2p[2] = neighbors[j].p_z - currentPt.p_z;
		double h_dis = -o2p[0]*currentPt.normVector[0] - o2p[1]*currentPt.normVector[1] - o2p[2]*currentPt.normVector[2];
		one_feature_vector.h_dis = abs(h_dis);
		
		//step5 计算距离op
		double d_dis = sqrt(o2p[0]* o2p[0] + o2p[1] * o2p[1] + o2p[2] * o2p[2]);
		one_feature_vector.d_dis = abs(d_dis);

		//step6 计算投影距离l_dis
		double o2proj[3];
		o2proj[0] = o2p[0] + h_dis*currentPt.normVector[0];
		o2proj[1] = o2p[1] + h_dis*currentPt.normVector[1];
		o2proj[2] = o2p[2] + h_dis*currentPt.normVector[2];
		double l_dis = sqrt(o2proj[0]* o2proj[0] + o2proj[1] * o2proj[1] + o2proj[2] * o2proj[2]);
		one_feature_vector.l_dis = abs(l_dis);

		//两个曲率相关的变量
		//step7 
		one_feature_vector.curvature = neighbors[j].curvature;

		//**step8 push back curvedness
		double curvedness = sqrt(neighbors[j].k1Cur*neighbors[j].k1Cur + neighbors[j].k2Cur*neighbors[j].k2Cur);
		one_feature_vector.curvedness = curvedness;

		feature_vectors.push_back(one_feature_vector);
	}

	
	
	const double epsilon = 1e-9; // 定义一个极小值，用于避免除以零

	
	auto normalize_feature = [&](double FEATURE_VECTOR::* feature_ptr) 
	{
		double max_val = ZYFMIN;
		double min_val = ZYFMAX;
		for (const auto& fv : feature_vectors) 
		{
			if (fv.*feature_ptr > max_val) max_val = fv.*feature_ptr;
			if (fv.*feature_ptr < min_val) min_val = fv.*feature_ptr;
		}

		if (std::abs(max_val - min_val) > epsilon) 
		{
			for (auto& fv : feature_vectors) 
			{
				fv.*feature_ptr = (fv.*feature_ptr - min_val) / (max_val - min_val);
			}
		}
		else 
		{
			for (auto& fv : feature_vectors) 
			{
				fv.*feature_ptr = 0.0; // 如果所有值都相同，归一化为0
			}
		}
	};

	// 对所有8个特征调用归一化函数
	normalize_feature(&FEATURE_VECTOR::aph);
	normalize_feature(&FEATURE_VECTOR::betta);
	normalize_feature(&FEATURE_VECTOR::kappa);
	normalize_feature(&FEATURE_VECTOR::d_dis);
	normalize_feature(&FEATURE_VECTOR::h_dis);
	normalize_feature(&FEATURE_VECTOR::l_dis);
	normalize_feature(&FEATURE_VECTOR::curvature);
	normalize_feature(&FEATURE_VECTOR::curvedness);

	
	double tempcov[8][8] = { 0.0 };
	double sum_weight = 0.0;
	for (int j = 0; j < neighbors.size(); j++)
	{
		double temp_weight = neighbors[j].w_interpolation;
		sum_weight += temp_weight;

		double features[8] = 
		{
			feature_vectors[j].aph,
			feature_vectors[j].betta,
			feature_vectors[j].kappa,
			feature_vectors[j].d_dis,
			feature_vectors[j].h_dis,
			feature_vectors[j].l_dis,
			feature_vectors[j].curvature,
			feature_vectors[j].curvedness
		};

		
		for (int r = 0; r < 8; ++r) 
		{
			for (int c = 0; c < 8; ++c) 
			{
				tempcov[r][c] += temp_weight * features[r] * features[c];
			}
		}
	}


	if (std::abs(sum_weight) > epsilon) 
	{
		for (int ii = 0; ii < 8; ii++) 
		{
			for (int jj = 0; jj < 8; jj++) 
			{
				tempcov[ii][jj] = tempcov[ii][jj] / sum_weight;
			}
		}
	}
	

	for (int ii = 0; ii < 8; ii++) 
	{
		for (int jj = 0; jj < 8; jj++) 
		{
			currentPt.covM[ii][jj] = tempcov[ii][jj];
		}
	}


	Eigen::Map<Eigen::Matrix<double, 8, 8, Eigen::RowMajor>> final_covM(currentPt.covM[0]);

	if (!final_covM.allFinite()) 
	{
		fprintf(stderr,"\n[WARNING in calabhAndFormMatrix] Final check failed for point index %d. Resetting covM to Identity.\n",currentPt.ownIndex);

	}
}