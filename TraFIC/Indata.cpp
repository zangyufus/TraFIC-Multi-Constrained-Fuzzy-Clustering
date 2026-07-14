#include "stdafx.h"
#include "Indata.h"
#include "divideStr.h"

CIndata::CIndata()
{
}

CIndata::~CIndata()
{
}

void CIndata::readsamplingPts(char* infile, std::vector<POINT_STRUCT>& ptsArray)
{
	FILE* fpIn = NULL;
	size_t err = fopen_s(&fpIn, infile, "r");
	if (err != 0)
	{
		printf("open file %s fail!\n", infile);
		return;
	}

	char buffer[512];
	while (fgets(buffer, 512, fpIn))
	{
		POINT_STRUCT pt;
		sscanf_s(buffer, "%lf%lf%lf", &pt.p_x, &pt.p_y, &pt.p_z);

		ptsArray.push_back(pt);
	}


	fclose(fpIn);
}





void CIndata::run(const char* infile, std::vector<POINT_STRUCT>& ptsArray)
{
	FILE* fpIn = NULL;
	size_t err = fopen_s(&fpIn, infile, "r");
	if (err != 0)
	{
		printf("open file %s fail!\n", infile);
		return;
	}
	
	//1.read head
	long int vertexNum = 0;
	long int element_face = 0;
	char buffer[512];
	while (fgets(buffer, 512, fpIn))
	{
		CdivideStr divideStr;
		//空间申请小了 就会内存不足
		char** ptr_array = (char**)malloc(sizeof(char*) * 50);
		divideStr.divideStr(buffer, ptr_array);

		if (strcmp(ptr_array[1], "element") == 0 && strcmp(ptr_array[2], "vertex") == 0)
		{
			vertexNum = atoi(ptr_array[3]);
		}

		if (strcmp(ptr_array[1], "element") == 0 && strcmp(ptr_array[2], "face") == 0)
		{
			element_face = atoi(ptr_array[3]);
		}

		if (strcmp(ptr_array[1], "end_header") == 0)
		{
			break;
		}
	}

	//2.read data
	long int nPts = 0;
	while (fgets(buffer, 512, fpIn))
	{
		POINT_STRUCT pt;
		size_t r, g, b;
		sscanf_s(buffer, "%lf%lf%lf%d%d%d", &pt.p_x, &pt.p_y, &pt.p_z, &r, &g, &b);
		pt.ownIndex = nPts;
		
		ptsArray.push_back(pt);
		nPts++;
		if (nPts == vertexNum)
		{
			break;
		}
	}

	//long size_t ntins = 0;
	//while (fgets(buffer, 512, fpIn))
	//{
	//	TIN tin;
	//	sscanf(buffer, "%*d%ld%ld%ld", &tin.id1, &tin.id2, &tin.id3);
	//	tin.id = ntins;
	//	tinsArray.push_back(tin);

	//	ntins++;
	//	if (ntins == element_face)
	//	{
	//		break;
	//	}
	//}


	fclose(fpIn);
}
