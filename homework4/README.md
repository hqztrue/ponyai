# Homework4

In this homework, you will learn how to fuse Lidar data with camera image and get familiar with perception output, which will be helpful for your final project.

## Homework

### 1. Re-implement the simple lane boundary detection.
Let's implement simple lane boundary detection based on the method mentioned in last lecture! Remember the steps? First convert the RGB image to greyscale, then try to use Canny edge detector to find those boundaries. Finally try Hough transform to fit those straight lines. Luckily, OpenCV provides APIs for both [Canny Edge detector](https://docs.opencv.org/2.4/doc/tutorials/imgproc/imgtrans/canny_detector/canny_detector.html) and [Hough transform](https://docs.opencv.org/2.4/doc/tutorials/imgproc/imgtrans/hough_lines/hough_lines.html). Try them on the dataset we provide to you in **homework1**.

**What to submit:** Your C++ code to detect simple lane boundary based on above method.

### 2. Project Pointcloud to image.
There are two main steps for projecting the Pointcloud to image. First, project Pointcloud from Lidar local coordinate system to camera coordinate system  and then project 3D points from camera coordinate system to image plane. 
Please implement functions **Project3dPointToImage** and **ProjectPointCloudToImage** in file **camera_lidar_fusion_utils.cc**. 
Read code comments carefully before writing your code. We also provide you a simple viewer to visualize the fusion result, you can use it with the data provided along with this homework.

`bazel run -c opt //homework4:camera_lidar_fusion_main  -- --pony_data_dir PONY_DATA_DIR`

Once you have the correct implementation, you should see a similar result as follows, **NOTE** the color of projected points is based on its depth, please read the code in **camera_lidar_fusion_main.cc** if you are interested.

_*Press any key to display the next frame._
![alt text](https://github.com/ponyai/PublicCourse/blob/master/homework4/fusion.png)

**What to submit:** Your implementaion of above two functions.

### 3. Extract the obstacle points from Pointcloud and create PerceptionObstacle.
In this homework, we also provide you some object label (Defined in **interface.object_labeling.ObjectLabels**) files which you can use the following cmd to visualize:

`bazel run -c opt //homework4:pointcloud_viewer_main -- --pony_data_dir PONY_DATA_DIR`

_*Press 'n' to display the next frame._
![alt text](https://github.com/ponyai/PublicCourse/blob/master/homework4/pointcloud_viewer.png)
**NOTE, you can also use this viewer to visualize the Pointcloud data in Homework1, but you need to manually move all folders with device name to a parent folder named with 'select', sorry about this inconvenience.**

Let's first get familiar with **interface.object_labeling.ObjectLabels** and **interface.perception.PerceptionObstacle** proto definitions. Your task is to extract object points from Pointcloud based on given labeled bounding box and fill them into proper field in **interface.perception.PerceptionObstacle**. Values for the other fields in this proto should be similar as they are in the label proto. Feel free to modify the viewer code to visualize the obstacle points or project them onto images in order to verify your implementation.

**What to submit:** Your C++ code to extract obstacle points and generate a list of **interface.perception.PerceptionObstacle** from each Pointcloud.
