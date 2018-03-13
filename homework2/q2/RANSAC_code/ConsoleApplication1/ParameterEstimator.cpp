#include "ParameterEstimator.h"
#include "Matrix.h"
void PlaneParamEstimator::estimate(vector<point3D*> &data, vector<double> &parameters){
	parameters.clear();
	if (data.size() < 3)return;
	point3D n = ((*data[1] - *data[0]) ^ (*data[2] - *data[0])).unit();
	parameters.push_back(n.x);
	parameters.push_back(n.y);
	parameters.push_back(n.z);
	parameters.push_back(data[0]->x);
	parameters.push_back(data[0]->y);
	parameters.push_back(data[0]->z);
}

void PlaneParamEstimator::leastSquaresEstimate(vector<point3D*> &data, vector<double> &parameters){
	//printf("s%d\n", data.size());
	/*puts("--------------");
	for (int i = 0; i < data.size(); ++i){
		printf("matrix %.5lf %.5lf %.5lf\n",data[i]->x, data[i]->y, data[i]->z);
	}*/
	double Y[3];
	double A, B, C;
	A = B = C = 0.0;

	matrix<3> Matrix, IMatrix;

	for (int i = 0; i < 3; i++)Y[i] = 0.0;

	//Matrix.print();
	for (int i = 0; i < data.size(); i++)
	{
		*(Matrix.key[0]) += data[i]->x*data[i]->x;
		*(Matrix.key[1]) += data[i]->x*data[i]->y;
		*(Matrix.key[2]) += data[i]->x;
		
		*(Matrix.key[0] + 1) += data[i]->x*data[i]->y;
		*(Matrix.key[1] + 1) += data[i]->y*data[i]->y;
		*(Matrix.key[2] + 1) += data[i]->y;

		*(Matrix.key[0] + 2) += data[i]->x;
		*(Matrix.key[1] + 2) += data[i]->y;
		*(Matrix.key[2] + 2) += 1;

		Y[0] += data[i]->x * data[i]->z;
		Y[1] += data[i]->y * data[i]->z;
		Y[2] += data[i]->z;
	}
	
	double d = Matrix.det();
	//printf("d=%e\n", d);
	if (fabs(d) < 0.0001)
	{
		printf("singular matrix\n");
		return;
	}
	//Matrix.print();
	IMatrix = Matrix.inverse();
	//IMatrix.print();
	//(Matrix * IMatrix).print();
	//for (int i = 0; i < 3; ++i)printf("Y%.8lf ", Y[i]);
	//printf("\n");

	for (int i = 0; i < 3; i++)
	{
		A += *(IMatrix.key[0] + i)*Y[i];
		B += *(IMatrix.key[1] + i)*Y[i];
		C += *(IMatrix.key[2] + i)*Y[i];
	}

	//double norm = sqrt(A*A + B*B + 1);

	parameters.push_back(A);
	parameters.push_back(B);
	parameters.push_back(-1);
	//parameters.push_back(C);

	double meanX, meanY, meanZ;
	meanX = meanY = meanZ = 0.0;
	for (int i = 0; i < data.size(); i++)
	{
		meanX += data[i]->x;
		meanY += data[i]->y;
		meanZ += data[i]->z;
	}

	meanX /= data.size();
	meanY /= data.size();
	meanZ /= data.size();

	parameters.push_back(meanX);
	parameters.push_back(meanY);
	parameters.push_back(meanZ);

	//puts("parameters");
	//for (int i = 0; i < parameters.size(); ++i)printf("%.6lf ", parameters[i]);
	//puts("");
}

bool PlaneParamEstimator::agree(vector<double> &parameters, point3D &data){
	double signedDistance = parameters[0] * (data.x - parameters[3]) + parameters[1] * (data.y - parameters[4]) + parameters[2] * (data.z - parameters[5]);
	//printf("%.5lf %.5lf %.5lf\n", data.x, data.y, data.z);
	//printf("%.5lf\n", fabs(signedDistance));
	return (signedDistance * signedDistance) < m_deltaSquared;
}


