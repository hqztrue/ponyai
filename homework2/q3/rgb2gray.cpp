// ConsoleApplication6.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include "cv.h"
#include "highgui.h"

#include<stdio.h>
#include<stdlib.h>
#include<iostream>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

Mat image, gray, gray_std;



int main(int argc, const char** argv)
{
	string filename = "d:\\0.jpg";

	image = imread(filename);
	if (image.empty())
	{
		printf("Cannot read image file: %s\n", filename.c_str());
		return -1;
	}
	cvtColor(image, gray_std, COLOR_BGR2GRAY);
	imwrite("D:\\2.jpg", gray_std);

	/*for (int i = 0; i < image.rows; ++i)
		for (int j = 0; j < image.cols; ++j){
			image.at<Vec3b>(i, j) = Vec3b(0, 0, 255);
		}
	imwrite("D:\\3.jpg", image);
	return 0;*/

	gray = Mat(image.size(), CV_8UC1);
	for (int i = 0; i < image.rows; ++i)
		for (int j = 0; j < image.cols; ++j){
			Vec3b col = image.at<Vec3b>(i, j);
			gray.at<uchar>(i, j) = 0.1140 * col[0] + 0.5870 * col[1] + 0.2989 * col[2];  //bgr; matlab's (NTSC/PAL) implementation
		}
	
	imwrite("D:\\3.jpg", gray);
	return 0;
}
