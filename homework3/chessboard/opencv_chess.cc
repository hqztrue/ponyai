// ConsoleApplication6.cpp : 定义控制台应用程序的入口点。
//

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include "opencv2/core/utility.hpp"
//#include "opencv2/imgproc.hpp"
//#include "opencv2/imgcodecs.hpp"
//#include "opencv2/highgui.hpp"

#include <stdio.h>

using namespace cv;
using namespace std;


double k1 = 0.1, k2 = 0.1;
void solve(double x1, double y1, double &x_, double &y_){
	double l = 0, r = 100, eps = 1e-10;
	if (fabs(x1) < eps) x1 += eps;
	while (r - l > eps){
		double x = (l + r) / 2, y = x/x1*y1, r2=x*x+y*y;
		if (x*(1 + k1*r2 + k2*r2*r2) < fabs(x1))l = x;
		else r = x;
	}
	x_ = (l + r) / 2;
	if (x1 < 0)x_ *= -1;
	y_ = x_/x1*y1;
	//printf("%.8lf\n", x_);
}

int main(int argc, const char** argv)
{
	Mat img;
	// ATTENTION!!! : please use absolute path for reading the data file.
	img = imread("/home/hqz/ponyai/homework3/chessboard/chessboard_undistorted.png", CV_LOAD_IMAGE_COLOR);
	//img = imread("chessboard_undistorted.png", CV_LOAD_IMAGE_COLOR);
	Mat distorted = Mat::zeros(img.rows, img.cols, CV_8UC3);
	/*for (int i = 0; i < img.rows; ++i)
		for (int j = 0; j < img.cols; ++j){
			double z = 350, x = i - 0.5*(img.rows - 1), y = j - 0.5*(img.cols - 1);
			x /= z, y /= z;
			double r2 = x*x + y*y;
			double x1 = x*(1 + k1*r2 + k2*r2*r2)*z, y1 = y*(1 + k1*r2 + k2*r2*r2)*z;
			int x1_ = int(round(x1 + 0.5*(img.rows - 1))), y1_ = int(round(y1 + 0.5*(img.cols - 1)));
			if (x1_>=0 && x1_<img.rows && y1_>=0 && y1_<img.cols)
				distorted.at<Vec3b>(x1_, y1_) = img.at<Vec3b>(i, j);
		}*/
	for (int i = 0; i < img.rows; ++i)
		for (int j = 0; j < img.cols; ++j){
			double z = 350, x = i - 0.5*(img.rows - 1), y = j - 0.5*(img.cols - 1);
			double x1 = 0, y1 = 0;
			solve(x/z, y/z, x1, y1);
			x1 *= z; y1 *= z;
			int x1_ = int(round(x1 + 0.5*(img.rows - 1))), y1_ = int(round(y1 + 0.5*(img.cols - 1)));
			//printf("%d %d\n", x1_, y1_);
			if (x1_ >= 0 && x1_<img.rows && y1_ >= 0 && y1_<img.cols)
				distorted.at<Vec3b>(i, j) = img.at<Vec3b>(x1_, y1_);
		}
	imwrite("/home/hqz/ponyai/homework3/chessboard/distorted.png", distorted);
	//system("pause");
	return 0;
}

