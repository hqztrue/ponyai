package(default_visibility = ["//visibility:public"])

cc_library(
    name = "perception",
    srcs = ["perception.cc"],
    hdrs = ["perception.h"],
    deps = [
        "//common/proto:cc_perception_proto",
        "//common/utils/common:defines",
        "//common/utils/common:optional",
        "//homework2:pointcloud",
        "@opencv//:core",
        "@opencv//:highgui",
    ],
)

cc_library(
    name = "perception_evaluator",
    srcs = ["perception_evaluator.cc"],
    hdrs = ["perception_evaluator.h"],
    deps = [
        ":perception",
        "//common/proto:cc_object_labeling_3d_proto",
        "//common/proto:cc_perception_evaluation_proto",
        "//common/utils/common:optional",
        "//common/utils/evaluation:grading",
        "//common/utils/file",
        "//common/utils/file:path",
        "//common/utils/hungarian:hungarian_sparse",
        "//common/utils/math",
        "//common/utils/math/transform",
        "//homework2:pointcloud",
        "@gflags//:gflags",
        "@glog//:glog",
        "@opencv//:core",
        "@opencv//:highgui",
    ],
)

cc_binary(
    name = "perception_evaluation_main",
    srcs = ["perception_evaluation_main.cc"],
    deps = [
        ":perception_evaluator",
        "//common/utils/file:path",
        "@gflags//:gflags",
    ],
)

cc_library(
    name = "evaluation_viewer",
    srcs = ["evaluation_viewer.cc"],
    hdrs = ["evaluation_viewer.h"],
    deps = [
        ":perception_evaluator",
        "//common/utils/display:painter_widget_base",
        "//common/utils/file:path",
        "@gflags//:gflags",
        "@glog//:glog",
        "@qt//:qt_widgets",
    ],
)

cc_binary(
    name = "evaluation_viewer_main",
    srcs = ["evaluation_viewer_main.cc"],
    deps = [
        ":evaluation_viewer",
        "//common/utils/file:path",
        "@glog//:glog",
        "@qt//:qt_widgets",
    ],
)
