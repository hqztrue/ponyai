#include "stdafx.h"
#include "common.h"
#include "util.h"
#include "ParameterEstimator.h"
#include "Ransac.h"

const int N = 1000005;
double array[N][3];
int r[N], g[N], b[N];
point3D trans(point3D p, vector<double> planeParameters){
	point3D w = point3D(planeParameters[0], planeParameters[1], planeParameters[2]).unit();
	//printf("%.5lf\n", p*w);
	p -= point3D(planeParameters[3], planeParameters[4], planeParameters[5]);
	point3D wo = (w.x<0.001) || (w.x > 0.001) ? point3D(0, 1, 0) : point3D(1, 0, 0);
	point3D u = (wo^w).unit();
	point3D v = (w^u).unit();
	//printf("coord %.5lf %.5lf %.5lf\n", w*u, w*v, u*v);
	return point3D(p*u, p*v, p*w);
}
int _tmain(int argc, _TCHAR* argv[])
{
	//srand(time(0));
	double newX, newY, newZ;

	/*int n = 10000;
	double A = 1, B = 2, C = 3, X = 13, Y = -888, Z = 123;
	
	double noiseSpreadRadius = 100;
	for (int i = 0; i < n; i++)
	{
		array[i][0] = rand() + noiseSpreadRadius*(double)rand() / (double)RAND_MAX * (rand() > RAND_MAX / 2 ? 1 : -1);
		array[i][1] = rand() + noiseSpreadRadius*(double)rand() / (double)RAND_MAX * (rand() > RAND_MAX / 2 ? 1 : -1);
		array[i][2] = (60 - A*array[i][0] - B*array[i][1]) / C + noiseSpreadRadius*(double)rand() / (double)RAND_MAX * (rand() > RAND_MAX / 2 ? 1 : -1);
	}*/



	FILE *fin = fopen("D:\\1.txt", "r");
	int n = 0, n1 = 0;
	//int r = 255, g = 0, b = 0;
	while (1){
		int id = 0;
		if (fscanf(fin, "%d,%lf,%lf,%lf", &id, &array[n][0], &array[n][1], &array[n][2]) == EOF)break;
		//printf("in %lf %lf %lf\n", array[n][0], array[n][1], array[n][2]);
		//if ((array[n][1] > 500) && (array[n][2] > 2800))--n;
		++n;
		//++n1;
		//if (n1%100==0)++n;
		//if (n > 1000)break;
		//printf("%d\n", n);
	}
	fclose(fin);
	//puts("sta");
	//for (;;);

	vector<double> planeParameters;
	vector<point3D> pointData;
	vector<point3D *> pointDataPtr;

	for (int i = 0; i < n; i++)
	{
		newX = array[i][0];
		newY = array[i][1];
		newZ = array[i][2];
		pointDataPtr.push_back(new point3D(newX, newY, newZ));
		pointData.push_back(*(pointDataPtr[pointDataPtr.size() - 1]));
	}
	printf("n=%d\n", n);

	PlaneParamEstimator lpEstimator(1);  //lpEstimator(0.1);
	int numForEstimate = 3;

	//double usedData = Ransac<point3D, double>::compute(planeParameters, &lpEstimator, pointData, numForEstimate);
	short *bestVotes = NULL;
	double usedData = Ransac<point3D, double>::compute(planeParameters, &lpEstimator, pointData, bestVotes, numForEstimate, 0.01, 0.99);
	
	FILE *fout = fopen("D:\\2.txt", "w");
	for (int i = 0; i < n; ++i){
		point3D p = trans(point3D(array[i][0], array[i][1], array[i][2]), planeParameters);
		//printf("bestVotes%d %d\n", bestVotes, bestVotes[i]);
		if (bestVotes[i]){
			//printf("%.5lf %.5lf %.5lf %.5lf %.5lf %.5lf\n", array[i][0], array[i][1], array[i][2], p.x, p.y, p.z);
			b[i] = 0; g[i] = 0; r[i] = 255;
		}
		else {
			b[i] = 255; g[i] = 0; r[i] = 0;
		}
		//r[i] = g[i] = b[i] = int(fabs(p.z) / 10);
		/*if (fabs(p.z - 1600) < 20){
			r[i] = 0; g[i] = 0; b[i] = 255;
		}*/
		fprintf(fout, "%.5lf %.5lf %.5lf %d %d %d\n", p.x, p.y, p.z, b[i], g[i], r[i]);
	}
	/*{
		for (int i = -10000; i < 10000; ++i){
			point3D w = point3D(planeParameters[0], planeParameters[1], planeParameters[2]).unit();
			point3D p = w*i;
			fprintf(fout, "%.5lf %.5lf %.5lf %d %d %d\n", p.x, p.y, p.z, 255, 255, 0);
		}
	}*/
	printf("trans %.5lf %.5lf %.5lf\n", planeParameters[0], planeParameters[1], planeParameters[2]);
	fclose(fout);
	cout << "RANSAC plane parameters [A, B, C, X0, Y0, Z0]\n" << "[" << planeParameters[0] << ", " << planeParameters[1] << ", "
		<< planeParameters[2] << ", " << planeParameters[3] << ", "
		<< planeParameters[4] << ", " << planeParameters[5] << "]\n";
	cout << "Percentage of points which were used for final estimate: " << usedData << endl;
	//cout << point3D(A, B, C).unit() << endl;
	system("pause");
	return 0;
}


