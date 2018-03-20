#include <string>
using namespace std;
#include <opencv2/core/core.hpp>
#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>

#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
using namespace cv;
//#include <pcl/io/lzf_image_io.h>
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;
int user_data;
pcl::ModelCoefficients plane_coeff;
PointCloud::Ptr cloud(new PointCloud);
void viewerOneOff(pcl::visualization::PCLVisualizer& viewer){
	viewer.setBackgroundColor(1.0, 0.5, 1.0);
	pcl::PointXYZ o;
	o.x = 1.0;
	o.y = 0;
	o.z = 0;
	viewer.addSphere(o, 0.25, "sphere", 0);
	std::cout << "i only run once" << std::endl;
}

void viewerPsycho(pcl::visualization::PCLVisualizer& viewer)
{
	static unsigned count = 0;
	std::stringstream ss;
	ss << "Once per viewer loop: " << count++;
	viewer.removeShape("text", 0);
	viewer.addText(ss.str(), 200, 300, "text", 0);
	//viewer.addPolygon(cloud, 123,45,177);
	//char str[105];
	//sprintf(str, "%d", rand()*rand());
	//viewer.addPlane(plane_coeff, str);
	//FIXME: possible race condition here:
	user_data++;
}

const double camera_factor = 1000;
const double camera_cx = 325.5;
const double camera_cy = 253.5;
const double camera_fx = 518.0;
const double camera_fy = 519.0;
double rand_(double l, double r){
	return l + (r - l)*rand() / RAND_MAX;
}
int main()
{
	bool use_rgb = false;
	int num_imgs = 200;
	float xmin = 1e10, xmax = -1e10, ymin = 1e10, ymax = -1e10, zmin = 1e10, zmax = -1e10;
	vector<PointT> v;
	for (int i0 = 0; i0 < num_imgs; ++i0){
		cv::Mat rgb, depth;
		char path[105];
		sprintf(path, "E:\\×ÊÁÏ\\pony_data\\VelodyneDevice32c\\%d.txt", i0);
		FILE *fin = fopen(path, "r");
		//plane_coeff.values.resize(4);
		//for (int i = 0; i < 4; ++i)
		//	fscanf(fin, "%lf", &plane_coeff.values[i]);
		double translation[3];
		double rotation[3][3];

		if (!use_rgb){
			fscanf(fin, "%lf,%lf,%lf", &translation[0], &translation[1], &translation[2]);
			fscanf(fin, "%lf,%lf,%lf%lf,%lf,%lf%lf,%lf,%lf",
				&rotation[0][0], &rotation[0][1], &rotation[0][2],
				&rotation[1][0], &rotation[1][1], &rotation[1][2],
				&rotation[2][0], &rotation[2][1], &rotation[2][2]);
		}

		while (1){
			PointT p;
			int r = 255, g = 0, b = 0, index;
			if (!use_rgb){
				if (fscanf(fin, "%d,%f,%f,%f", &index, &p.x, &p.y, &p.z) == EOF)break;
				PointT p1 = p;
				p.x = rotation[0][0] * p1.x + rotation[0][1] * p1.y + rotation[0][2] * p1.z + translation[0];
				p.y = rotation[1][0] * p1.x + rotation[1][1] * p1.y + rotation[1][2] * p1.z + translation[1];
				p.z = rotation[2][0] * p1.x + rotation[2][1] * p1.y + rotation[2][2] * p1.z + translation[2];
			}
			else {
				if (fscanf(fin, "%f%f%f%d%d%d", &p.x, &p.y, &p.z, &b, &g, &r) == EOF)break;
			}
			//printf("%.5f %.5f %.5f\n", p.x, p.y, p.z);
			//if (fscanf(fin, "%f%f%f", &p.x, &p.y, &p.z) == EOF)break;
			xmin = min(xmin, p.x);
			xmax = max(xmax, p.x);
			ymin = min(ymin, p.y);
			ymax = max(ymax, p.y);
			zmin = min(zmin, p.z);
			zmax = max(zmax, p.z);
			p.b = b; p.g = g; p.r = r;
			v.push_back(p);
			//printf("%.5f %d\n", p.x, (int)p.r);
		}
	}
	printf("z: %.5lf %.5lf\n", zmin, zmax);
	Mat height = Mat::zeros(floor((xmax - xmin) / 0.5) + 1, floor((ymax - ymin) / 0.5) + 1, CV_32F);
	for (int i = 0; i < height.rows; ++i)
		for (int j = 0; j < height.cols; ++j)
			height.at<float>(i, j) = zmax;
	for (int i = 0; i < v.size(); ++i){
		v[i].x -= xmin;
		v[i].y -= ymin;
		int x1 = floor(v[i].x / 0.5), y1 = floor(v[i].y / 0.5);
		height.at<float>(x1, y1) = min(height.at<float>(x1, y1), v[i].z);
		//cloud->points.push_back(v[i]);
	}
	{
		Mat height1 = Mat::zeros(height.rows, height.cols, CV_8UC1);
		for (int i = 0; i < height.rows; ++i)
			for (int j = 0; j < height.cols; ++j)
				height1.at<uchar>(i, j) = 255 * (height.at<float>(i, j)-zmin) / (zmax - zmin);
		imwrite("height.png", height1);
	}
	vector<float> vecz;
	for (int i = 0; i < height.rows; ++i)
		for (int j = 0; j < height.cols; ++j)
			if (height.at<float>(i, j) < zmax)
				vecz.push_back(height.at<float>(i, j));
	int ratio = 0.6 * vecz.size();
	nth_element(vecz.begin(), vecz.begin() + ratio, vecz.end());
	float z0 = vecz[ratio];
	printf("split z: %.5lf\n", z0);
	for (int i = 0; i < v.size(); ++i){
		if (v[i].z<z0)cloud->points.push_back(v[i]);
	}

	/*for (int i = 1; i <= 10000; ++i){
		double x = rand_(xmin, xmax), y = rand_(ymin, ymax);
		double z = -(plane_coeff.values[0] * x + plane_coeff.values[1] * y + plane_coeff.values[3]) / plane_coeff.values[2];
		PointT p;
		p.x = x; p.y = y; p.z = z;
		p.r = 255; p.g = 0; p.b = 0;
		cloud->points.push_back(p);
	}*/
	cout << "point cloud size = " << cloud->points.size() << endl;
	cloud->height = 1;
	cloud->width = cloud->points.size();
	cloud->is_dense = false;
	pcl::io::savePCDFile("pointcloud.pcd", *cloud);

	pcl::visualization::CloudViewer viewer("Cloud Viewer");
	viewer.showCloud(cloud);
	viewer.runOnVisualizationThreadOnce(viewerOneOff);
	viewer.runOnVisualizationThread(viewerPsycho);
	cloud->points.clear();
	cout << "Point cloud saved." << endl;
	/*
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::io::loadPCDFile ("my_point_cloud.pcd", *cloud);

	pcl::visualization::CloudViewer viewer("Cloud Viewer");

	//blocks until the cloud is actually rendered
	viewer.showCloud(cloud);

	//use the following functions to get access to the underlying more advanced/powerful
	//PCLVisualizer

	//This will only get called once
	viewer.runOnVisualizationThreadOnce (viewerOneOff);

	//This will get called once per visualization iteration
	viewer.runOnVisualizationThread (viewerPsycho);
	*/
	while (!viewer.wasStopped())
	{
		//you can also do cool processing here
		//FIXME: Note that this is running in a separate thread from viewerPsycho
		//and you should guard against race conditions yourself...
		user_data++;
	}
	return 0;
}
