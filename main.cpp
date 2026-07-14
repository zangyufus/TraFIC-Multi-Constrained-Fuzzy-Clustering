#include "stdafx.h"
#pragma once
#include <time.h>
#include<stdio.h>
#include <vector>
#include <iostream>
#include <fstream>  
#include <iomanip> 
#include <pcl/io/pcd_io.h>
#include<string>
#include <iomanip>  
#include <limits>
#include "Indata.h"
#include "BAMatrix.h"
#include "Outdata.h"
#include "KnnSearch.h"
#include "CalEigen.h"
#include "ConsistPropagate.h"
#include "CalCurvature.h"
#include "Registration.h"
#include "Base.h"
#include "INys.h"
#include "IO.h"
#include"kmeans.h"
#include"RegistrationStruct.h"
#include"align_to_nearest_average.h"
#include "CoordinateCategoryConstraint.h" 
#include "RegistrationStruct.h"
#include "CoordinateCategoryConstraint.h" 
#include "DescriptorUtils.h"
#include "DataLoader.h"
using namespace std;

int main() {
	clock_t start = clock();
	//  STEP 1: 加载点云 
	printf("--- Loading all point clouds ---\n");
	CIndata indata;
	std::vector<POINT_STRUCT> src_original, tar_original;
	indata.run("deformed-TG6-2-1.ply", src_original);
	indata.run("TG6-2.ply", tar_original);

	printf("--- Loading sparse sample clouds ---");
	std::vector<POINT_STRUCT> src_sampled, tar_sampled;
	loadLabledPlyFile("deformed-TG6-2-1-sub.ply", src_sampled);
	loadLabledPlyFile("TG6-2-sub.ply", tar_sampled);

	printf("\n---  STEP1 DONE  ---\n");

	//  STEP 2: 
	printf("\n--- Precomputing Descriptors for Source Sampled Points ---\n");
	computeDescriptorsForSampledPoints(src_original, src_sampled);

	printf("\n--- Precomputing Descriptors for Target Sampled Points ---\n");
	computeDescriptorsForSampledPoints(tar_original, tar_sampled);

	printf("\n---  STEP2 DONE  ---\n");

	// STEP 3: 释放不再需要的密集点云内存 
	printf("\nReleasing memory from original point clouds...\n");
	std::vector<POINT_STRUCT>().swap(src_original);
	std::vector<POINT_STRUCT>().swap(tar_original);
	printf("Memory released.\n");
	printf("\n---  STEP3 DONE  ---\n");


	//STEP 4: 计算两种权重矩阵 
	printf("\n--- Calculating Weights ---\n");

	// 4.1 计算类别约束权重 
	printf("Setting up category constraints from point labels...\n");

	
	std::vector<int> src_categories, tar_categories;
	src_categories.reserve(src_sampled.size());
	tar_categories.reserve(tar_sampled.size());
	for (const auto& p : src_sampled) 
	{
		src_categories.push_back(p.category);
	}
	for (const auto& p : tar_sampled) 
	{
		tar_categories.push_back(p.category);
	}
	printf("Category vectors extracted directly from point clouds.\n");

	
	CCoordinateCategoryConstraint categoryConstraint;

	
	categoryConstraint.printCategoryStatistics(src_categories, "Source Cloud (Sampled)");
	categoryConstraint.printCategoryStatistics(tar_categories, "Target Cloud (Sampled)");
	std::vector<std::vector<double>> category_weights;
	double same_category_weight = 1.0;
	double different_category_weight = 0.1;

                printf("Calculating category weight matrix...\n");
	categoryConstraint.calculateCategoryWeights(src_categories,tar_categories,category_weights,same_category_weight,different_category_weight);
	printf("Category constraint setup completed successfully.\n");
	printf("\n---  STEP4.1 CalCategory weights DONE  ---\n");

	// 4.2 计算描述子相似度权重
	std::vector<std::vector<double>> descriptor_similarity_weights;
	calculateSimilarityFromSampled(src_sampled, tar_sampled, src_categories,tar_categories, descriptor_similarity_weights);
	printf("\n---  STEP4 DONE  ---\n");
	//STEP 5

	
	combineWeightsInPlace(descriptor_similarity_weights, category_weights);

	// 释放不再需要的 category_weights
	std::vector<std::vector<double>>().swap(category_weights);
	printf("Memory for category weights released.\n");

	printf("\n---  STEP5 DONE  ---\n");
	// STEP 6: 执行
	printf("\n--- Starting Fuzzy Cluster Registration ---\n");

	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tar(new pcl::PointCloud<pcl::PointXYZ>());
	for (const auto& p : src_sampled) cloud_src->push_back(p.toPclPointXYZ());
	for (const auto& p : tar_sampled) cloud_tar->push_back(p.toPclPointXYZ());
	cloud_src->width = cloud_src->size(); cloud_src->height = 1;
	cloud_tar->width = cloud_tar->size(); cloud_tar->height = 1;

	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_res(new pcl::PointCloud<pcl::PointXYZ>());
	CRegistration reg;
	Eigen::RowVectorXd alpha;
	reg.fuzzy_cluster_reg(cloud_src, cloud_tar, alpha, cloud_res, descriptor_similarity_weights);

	printf("\n---  STEP6 DONE  ---\n");

	// ====================== 结束 ======================
	clock_t finish = clock();
	double duration = (double)(finish - start) / CLOCKS_PER_SEC;
	printf("\nRegistration finished.\nTotal execution time: %.2f seconds.\n", duration);

	system("pause");
	return 0;
}