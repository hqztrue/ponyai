# HomeWork2

Now you guys are familiar with the enviroment, let's get hands on the real data!

## New dependency: Eigen 
[Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) is a header only linear algebra library, which supports matrices, vectors, numerical solvers, and related algorithms. It will be used in multiple places through out the course. **Note:** You don't need to install this library by yourself, Bazel will do that for you.

## Homework

### 1. Get familiar with Pointcloud data structure
Use histogram to find out the distribution over point's **range** and **height**. The range is defined as the length of each point's 3d vector (origin is (0,0,0)) and height is simply its z value. A simple application has been provided to read the pointcloud from file (`bazel run -c opt //homework2:main`). You don't need to worry about the rotation and translation field of PointCloud for now, they will be used in later homeworks.

### 2. Separate the ground points from pointcloud(**Advanced**)
Ground detection is an important task for 3D lidar perception. Try to separate the ground from pointcloud by utilizing the characteristics of point distribution.

### 3. Image color manipulation
In the given sample data, the image is expressed in RGB color space. But in a lot of computer vision applications, we do need grey image. Find a proper way to convert the RGB image to grey image. Implement the color conversion by yourself and try not to use existing OpenCV APIs.

### 3. Lane boundary and marks detection(**Advanced**)
Try to use basic computer vision method to find oour lane boundaries and marks in images. OpenCV has provided a lot of feature detection [APIs](https://docs.opencv.org/2.4/modules/imgproc/doc/feature_detection.html). You may find some of them are very useful in this task.

