package(default_visibility = ["//visibility:public"])

cc_library(
    name = "pointcloud",
    srcs = ["pointcloud.cc"],
    hdrs = ["pointcloud.h"],
    deps = [
        "@eigen//:eigen",
        "@glog//:glog",
    ],
)

cc_binary(
    name = "main",
    srcs = ["main.cc"],
    deps = [
        ":pointcloud",
    ],
)
