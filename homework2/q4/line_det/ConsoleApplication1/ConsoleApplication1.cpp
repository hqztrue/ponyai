// ConsoleApplication1.cpp: 定义控制台应用程序的入口点。
//

#include "stdafx.h"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
using namespace cv;
using namespace std;

int main() {
	char path[105] = "d:\\0.jpg";
	Mat img = imread(path);
	Mat edge, cedge;
	//resize(img, img, Size(640, 480));
	//Canny(img, edge, 50, 200, 3); 
	//Canny(img, edge, 150, 350, 3);
	Canny(img, edge, 350, 600, 3);
	imwrite("edge.png", edge);
	//imshow("edge", edge);
	//waitKey(0);
	cvtColor(edge, cedge, CV_GRAY2BGR);

	/*vector<Vec2f> lines;
	HoughLines(edge, lines, 1, CV_PI / 180, 150, 0, 0);
	for (size_t i = 0; i < lines.size(); i++)
	{
		float rho = lines[i][0], theta = lines[i][1];
		Point pt1, pt2;
		double a = cos(theta), b = sin(theta);
		double x0 = a * rho, y0 = b * rho;
		pt1.x = cvRound(x0 + 1000 * (-b));
		pt1.y = cvRound(y0 + 1000 * (a));
		pt2.x = cvRound(x0 - 1000 * (-b));
		pt2.y = cvRound(y0 - 1000 * (a));
		line(cedge, pt1, pt2, Scalar(0, 0, 255), 3, CV_AA);
	}*/

	vector<Vec4i> lines;
	HoughLinesP(edge, lines, 1, CV_PI / 180, 100, 100, 5);
	//return 0;
	for (size_t i = 0; i < lines.size(); i++){
		Vec4i l = lines[i];
		double angle = 1.0*(l[3]-l[1])/(l[2]-l[0]);
		//if (fabs(angle)>0.3)  //some hard-coded parameter.
			line(img, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 3, CV_AA);
	}

	//imshow("lines", cedge);
	//waitKey(0);
	//imwrite("line.png", cedge);
	imwrite("line.png", img);
	return 0;
}

