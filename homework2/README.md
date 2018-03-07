# Homework2

Now you guys are familiar with the environment, let's get hands on the real data!

## New dependency: Eigen 
[Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) is a header only linear algebra library, which supports matrices, vectors, numerical solvers, and related algorithms. It will be used in multiple places throughout the course. **Note:** You don't need to install this library by yourself, Bazel will do that for you.

## Homework

### 1. Get familiar with Pointcloud data structure
Generally speaking, the Pointcloud is a collection of 3D points. Each point lies in Lidar's local coordinate system where the origin is defined as Lidar center and X axis is pointing forward. The rotation and translation to transform the pointcloud to world coordinate system are also provided. You don't need to worry about those fields for now, they will be covered in later lectures. A simple application has been provided to read the Pointcloud from file (`bazel run -c opt //homework2:main`).

Try to visualize the Pointcloud first and get familiar with the 3D world perceived by Lidar, since you will work on the Pointcloud a lot during this course. Our first task is using a histogram to find out the distribution over point's **range** and **height**. Here the range is defined as the length of each point's 3d vector (origin is (0,0,0)) and height is simply its z value. Try to plot those histograms and get deeper insight about how those lidar points distribute.

### 2. Separate the ground points from Pointcloud(**Open-ended task**)
Now you are familiar with our Pointcloud data and let's start to work on some more advanced tasks! In order to determine the real obstacles in Pointcloud, we first need to know which points belong to the ground and which are not. Ground detection is an important task for 3D lidar perception. Let's try to separate the ground from Pointcloud by utilizing the characteristics of point distribution. 
**Note, those are open-ended tasks, try your best to work out a reasonable solution for now and you will learn better method along with this course.**

### 3. Image color manipulation
In the given sample data, the image is expressed in RGB color space. But in a lot of computer vision applications, we do need grey image. Find a proper way to convert the RGB image to grey image. Implement the color conversion by yourself and try not to use existing OpenCV APIs.

### 3. Lane boundary and marks detection(**Open-ended task**)
Try to use basic computer vision method to find our lane boundaries and marks in images. OpenCV has provided a lot of feature detection [APIs](https://docs.opencv.org/2.4/modules/imgproc/doc/feature_detection.html). You may find some of them are very useful in this task.


