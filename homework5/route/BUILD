package(default_visibility = ["//visibility:public"])

cc_library(
    name = "find_route",
	srcs = ["find_route.cc"],
    hdrs = ["find_route.h", "geometry.h"],
    deps = [
        "//common/proto:cc_map_proto",
        "//common/utils/file",
		"//homework5/display:main_window",
        "//homework5/map:map_lib",
        "@gflags",
        "@glog",
    ],
)
