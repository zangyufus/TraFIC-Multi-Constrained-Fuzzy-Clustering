#include "stdafx.h"
#include "KnnSearch.h"


CKnnSearch::CKnnSearch()
{
}

CKnnSearch::~CKnnSearch()
{
}


void CKnnSearch::findsamePts(std::vector<POINT_STRUCT>& X_ptsArray, std::vector<POINT_STRUCT>& X_srcArray)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);


	cloud->width = X_srcArray.size();
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);

	for (size_t i = 0; i < cloud->points.size(); i++)
	{
		cloud->points[i].x = X_srcArray[i].p_x;
		cloud->points[i].y = X_srcArray[i].p_y;
		cloud->points[i].z = X_srcArray[i].p_z;
	}


	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;


	kdtree.setInputCloud(cloud);

	for (int i = 0; i < X_ptsArray.size(); i++)
	{

		pcl::PointXYZ searchPoint;
		searchPoint.x = X_ptsArray[i].p_x;
		searchPoint.y = X_ptsArray[i].p_y;
		searchPoint.z = X_ptsArray[i].p_z;

		
		std::vector<int> pointIdxNKNSearch(1);
		std::vector<float> pointNKNSquaredDistance(1);

		if (kdtree.nearestKSearch(searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
		{
			int index = pointIdxNKNSearch[0];
			X_ptsArray[i].normVector[0] = X_srcArray[index].normVector[0];
			X_ptsArray[i].normVector[1] = X_srcArray[index].normVector[1];
			X_ptsArray[i].normVector[2] = X_srcArray[index].normVector[2];

			X_ptsArray[i].curvature = X_srcArray[index].curvature;
		}
	}
}


void CKnnSearch::run(std::vector<POINT_STRUCT>& ptsArray)
{

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);


	cloud->width = ptsArray.size();
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);

	for (size_t i = 0; i < cloud->points.size(); i++)
	{
		cloud->points[i].x = ptsArray[i].p_x;
		cloud->points[i].y = ptsArray[i].p_y;
		cloud->points[i].z = ptsArray[i].p_z;
	}

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

	
	kdtree.setInputCloud(cloud);

	for (int i = 0; i < ptsArray.size(); i++)
	{
		
		pcl::PointXYZ searchPoint;
		searchPoint.x = ptsArray[i].p_x;
		searchPoint.y = ptsArray[i].p_y;
		searchPoint.z = ptsArray[i].p_z;

		
		std::vector<int> pointIdxNKNSearch(KNUM);
		std::vector<float> pointNKNSquaredDistance(KNUM);

		if (kdtree.nearestKSearch(searchPoint, KNUM, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
		{
			
			for (int j = 1; j < pointIdxNKNSearch.size(); j++)
			{
				int index = pointIdxNKNSearch[j];
				
				ptsArray[i].knnPts_array.push_back(index);
			}
		}
	}
}