#include "stdafx.h"
#include "Outdata.h"


COutdata::COutdata()
{
}


COutdata::~COutdata()
{
}

void COutdata::ouTransformedPly(char* outfile, CBAMatrix& R, CBAMatrix& T, std::vector<POINT_STRUCT> y_ptsArray)
{
	FILE* fp = NULL;
	int err = fopen_s(&fp, outfile, "w");
	if (err != 0)
		return;

	fprintf(fp, "ply\n");
	fprintf(fp, "format ascii 1.0\n");
	fprintf(fp, "comment VCGLIB generated\n");
	fprintf(fp, "element vertex %ld\n", y_ptsArray.size());
	fprintf(fp, "property float x\n");
	fprintf(fp, "property float y\n");
	fprintf(fp, "property float z\n");
	fprintf(fp, "property	uchar	red\n");
	fprintf(fp, "property	uchar	green\n");
	fprintf(fp, "property	uchar	blue\n");
	fprintf(fp, "element face %ld\n", 0);
	fprintf(fp, "property list uchar int vertex_indices\n");
	fprintf(fp, "end_header\n");

	for (int i = 0; i < y_ptsArray.size(); i++)
	{
		int r;	r = 0;
		int g;	g = 255;
		int b;	b = 0;

		//rotation and translation
		CBAMatrix one_Y;
		one_Y.alloc(3, 1);	one_Y.clear();
		one_Y.setElement(0, 0, y_ptsArray[i].p_x);
		one_Y.setElement(1, 0, y_ptsArray[i].p_y);
		one_Y.setElement(2, 0, y_ptsArray[i].p_z);

		CBAMatrix one_temp;
		one_temp.alloc(3, 1);	one_temp.clear();
		//CBAMatrix::MultiplyTranspose(R, one_Y, one_temp);
		CBAMatrix::Multiply(R, one_Y, one_temp);
		CBAMatrix::Plus(T, one_temp);
		fprintf(fp, "%lf\t%lf\t%lf\t%d\t%d\t%d\n", one_temp(0, 0), one_temp(1, 0), one_temp(2, 0), r, g, b);
	}

	fclose(fp);
}

void COutdata::outputPtsRenderedByNorm(char* filename, std::vector<POINT_STRUCT>& pointArray)
{
	FILE* fp = NULL;
	int err = fopen_s(&fp, filename, "w");
	if (err != 0)
	{
		printf("open file %s fail!\n", filename);
		return;
	}

	fprintf(fp, "ply\n");
	fprintf(fp, "format ascii 1.0\n");
	fprintf(fp, "element vertex %d\n", pointArray.size());
	fprintf(fp, "property float x\n");
	fprintf(fp, "property float y\n");
	fprintf(fp, "property float z\n");
	fprintf(fp, "property uchar diffuse_red\n");
	fprintf(fp, "property uchar diffuse_green\n");
	fprintf(fp, "property uchar diffuse_blue\n");
	fprintf(fp, "end_header\n");


	for (int i = 0; i < pointArray.size(); i++)
	{
		POINT_STRUCT& point = pointArray[i];
		fprintf(fp, "%lf\t%lf\t%lf\t", point.p_x, point.p_y, point.p_z);

		int n1 = 255 * ((point.normVector[0] + 1.0) / 2.0);
		int n2 = 255 * ((point.normVector[1] + 1.0) / 2.0);
		int n3 = 255 * ((point.normVector[2] + 1.0) / 2.0);


		fprintf(fp, "%d\t%d\t%d\n", n1, n2, n3);
	}

	fclose(fp);
}


void COutdata::outputPtsRenderedByCur(char* filename, std::vector<POINT_STRUCT> ptsArray)
{
	FILE* fp = NULL;
	int err = fopen_s(&fp, filename, "w");
	if (err != 0)
	{
		printf("open file %s fail!\n", filename);
		return;
	}

	fprintf(fp, "ply\n");
	fprintf(fp, "format ascii 1.0\n");
	fprintf(fp, "comment VCGLIB generated\n");
	fprintf(fp, "element vertex %ld\n", ptsArray.size());
	fprintf(fp, "property float x\n");
	fprintf(fp, "property float y\n");
	fprintf(fp, "property float z\n");
	fprintf(fp, "property uchar diffuse_red\n");
	fprintf(fp, "property uchar diffuse_green\n");
	fprintf(fp, "property uchar diffuse_blue\n");
	fprintf(fp, "element face %ld\n", 0);
	fprintf(fp, "property list uchar int vertex_indices\n");
	fprintf(fp, "end_header\n");


	double maxm, minm;
	maxm = ZYFMIN;
	minm = ZYFMAX;
	for (int i = 0; i < ptsArray.size(); i++)
	{
		if (ptsArray[i].curvature > maxm)
			maxm = ptsArray[i].curvature;

		if (ptsArray[i].curvature < minm)
			minm = ptsArray[i].curvature;
	}
	printf("max:%lf\n", maxm);
	printf("min:%lf\n", minm);


	maxm = 1.0;
	minm = 0.0;
	//maxm = 0.0001;
	//maxm = 0.03;
	//maxm = 2.0;

	double step = (maxm - minm) / 4.0;
	//min ~ mmin+step
	//mmin+step ~ mmin+2step
	//mmin+2step ~ mmin+3step
	//mmin+3step ~ mmin+4step

	int r, g, b;
	r = 0; g = 0; b = 0;
	for (int i = 0; i < ptsArray.size(); i++)
	{
		POINT_STRUCT& point = ptsArray[i];
		if (point.curvature <= minm)
		{
			r = 0; g = 0; b = 255;
		}
		if (minm < point.curvature && point.curvature <= minm + step)
		{
			double buffer = (point.curvature - minm) / step;
			r = 0;
			g = buffer * 255;
			b = 255;
		}
		if (minm + step < point.curvature &&  point.curvature <= minm + 2 * step)
		{
			double buffer = (point.curvature - minm - step) / step;
			r = 0;
			g = 255;
			b = 255 - buffer * 255;
		}
		if (minm + 2 * step < point.curvature && point.curvature <= minm + 3 * step)
		{
			double buffer = (point.curvature - minm - 2 * step) / step;
			r = buffer * 255;
			g = 255;
			b = 0;
		}
		if (minm + 3 * step < point.curvature && point.curvature <= maxm)
		{
			double buffer = (point.curvature - minm - 3 * step) / step;
			r = 255;
			g = 255 - buffer * 255;
			b = 0;
		}
		if (maxm < point.curvature)
		{
			r = 255; g = 0; b = 0;
		}

		fprintf(fp, "%lf\t%lf\t%lf\t%d\t%d\t%d\n", point.p_x, point.p_y, point.p_z, r, g, b);
	}


	fclose(fp);
}