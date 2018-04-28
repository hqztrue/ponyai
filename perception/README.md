# Perception Project

Congratulations for all of you to make it here. In our perception project, we will utilize those techniques we learnt
through this course to perform object detection task on the real world data. The quality of your solution will
be evaluated by using the metrics taught in the lecture.

## Goal
Implement the function `interface::perception::PerceptionObstacles RunPerception(const PointCloud& pointcloud, const utils::Optional<cv::Mat>& image)`
in `perception.cc`. The inputs for this function are Pointcloud from our Lidar sensor and images captured
by front camera. Your outputs should be all detected objects within certain range and organized by using
protobuf `interface::perception::PerceptionObstacles`, which you should already know well in Homework4.
The detection range is evaluation is defined as 30m, which means we will only evaluate your method within 30m
to the Lidar center. (which also means even you missed all objects beyond 30m, your evaluation score will 
not be affected at all, Hooray!)

**Note, the following field in `interface::perception::PerceptionObstacles` need to be filled corrected 
in order to be evaluated properly:**
- `interface.geometry.Point3D polygon_point`: All polygon points need to be in world coordinate system.
Only **x** and **y** are required be be correct and **z** will only affect visualization.

- `ObjectType type`: The type of the object (please refer `perception.proto` for all supported object
types and you only need to care about **UNKNOWN_TYPE**, **CAR** and **PEDESTRIAN** for this project.) Please
be aware that **UNKNOWN_TYPE** will be filtered out in evaluation so that they won't contribute to the score
directly. **Please note although we won't evaluate classification in this project, classification accuracy
will still affect the overall precision and recall.** For example, when you detect some object which doesn't
overlap with any existing label, if you classify it as **UNKNOWN_TYPE**, your overall precision will
not be affected. However, if you classify it as any other type, you will see a drop in calculating the
overall precision (See following examples for details.)

- The other fields, such as `double height` will only affect the visualization.

## Metric Calculation Example
Let's go through some concrete examples so you can have more sense about how those metrics are calculated.
The following figure shows a Pointcloud with 6 labels.

![alt text](https://github.com/ponyai/PublicCourseInternalReview/blob/perception_project/perception_project/figures/labels.png)
If you detect one car perfectly (Jaccard index between label and detected object is 1.0):

![alt text](https://github.com/ponyai/PublicCourseInternalReview/blob/perception_project/perception_project/figures/detect1.png)
The **overall precision** is 1.0 and the **overall recall** is 1/6 since only one label is detected 
out of six. If you detect two cars perfectly(almost) and one additional false positive car:

![alt text](https://github.com/ponyai/PublicCourseInternalReview/blob/perception_project/perception_project/figures/detect3.png)
The **overall precision** now becomes 2/3 and the **overall recall** is 2/6. You can notice that the false positive
car will reduce the **overall precision**. However, if you detect this additional object as **UNKNOWN_TYPE**, **overall precision** will go back to 1.0 and **overall recall** still keeps same.

## Run Perception Evaluation
First update the data and label path in `config/eval.config` (Please update all .config files when 
you want to run multiple scenarios), then run the following command:

```
bazel run -c opt //perception:perception_evaluation_main -- --evaluation_config_dir /${Path to your config foler}
```

If your config file and data folder are set properly, you will see the following output:

```
Evaluation results for /home/bocongl/workspace/data/Tsinghua/eval.config
/home/bocongl/workspace/data/Tsinghua/sample/select/VelodyneDevice32c/0.txt, [Precision:0.997159], [Recall: 0.332386]
/home/bocongl/workspace/data/Tsinghua/sample/select/VelodyneDevice32c/1.txt, [Precision:0.761632], [Recall: 0.253877]
/home/bocongl/workspace/data/Tsinghua/sample/select/VelodyneDevice32c/2.txt, [Precision:0.593439], [Recall: 0.197813]
/home/bocongl/workspace/data/Tsinghua/sample/select/VelodyneDevice32c/3.txt, [Precision:0.548458], [Recall: 0.182819]
/home/bocongl/workspace/data/Tsinghua/sample/select/VelodyneDevice32c/4.txt, [Precision:0.536], [Recall: 0.178667]
/home/bocongl/workspace/data/Tsinghua/sample/select/VelodyneDevice32c/5.txt, [Precision:0.529713], [Recall: 0.176571]
/home/bocongl/workspace/data/Tsinghua/sample/select/VelodyneDevice32c/6.txt, [Precision:0.525845], [Recall: 0.175282]
/home/bocongl/workspace/data/Tsinghua/sample/select/VelodyneDevice32c/7.txt, [Precision:1], [Recall: 0.166667]
/home/bocongl/workspace/data/Tsinghua/sample/select/VelodyneDevice32c/8.txt, [Precision:0.991304], [Recall: 0.165217]
/home/bocongl/workspace/data/Tsinghua/sample/select/VelodyneDevice32c/9.txt, [Precision:0.989583], [Recall: 0.164931]
/home/bocongl/workspace/data/Tsinghua/sample/select/VelodyneDevice32c/10.txt, [Precision:1], [Recall: 0.166667]


*********************************************************
* [Average Precision:0.7703], [Average Recall: 0.1964] *
*********************************************************
Evaluation result folder: /tmp/1524721168/
```
The program will calculate the overall precision and recall for each frame for each config file and 
output the averaged ones in the end. (All frames will have the same weight during averaging.)

## Run Perception Evaluation Viewer
In order to better understand the performance of your code, we also provide a evaluation viewer to help
you visualize the label and the object detected by your code. To run the viewer:

```
bazel run -c opt //perception:evaluation_viewer_main  -- --eval_result_file ${Evaluation result folder}/${scenario_name}.result
```

**You can find the evaluation result folder location at the end of `perception_evaluation_main` output.
and scenario_name is set in each .config file**

Once you run the command. You will see an user interface pop up and press **N** to load the next frame.
The label is colored in yellow, and your detected object is colored in red. use **L** or **D** to hide
one of them.

## Optional
Sometimes object detection purely based on Pointcloud is challenging even within 30m. You can try to
use the camera image to provide extra information! Modify your `config/eval.config` and add the camera
data path as follows:
```
local_data {
  data_dir: "/${DATA_DIR}/select/VelodyneDevice32c"
  label_dir: "/${DATA_DIR}/label/VelodyneDevice32c"
  camera_dir: "/${DATA_DIR}/select/GigECameraDeviceWideAngle"
}
```
Now run the perception evaluation again, you will notice the corresponding image is now available in 
`interface::perception::PerceptionObstacles RunPerception(const PointCloud& pointcloud, const utils::Optional<cv::Mat>& image)`
Recall what you have learned in Homework4, let's fuse both Lidar and camera data together and see
how that can help you in this task.
