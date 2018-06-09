package(default_visibility = ["//visibility:public"])

cc_library(
    name = "find_route",
	srcs = ["find_route.cc"],
    hdrs = ["find_route.h"],
    deps = [
        "//common/proto:cc_map_proto",
        "//common/utils/file",
		#"//homework6/display:main_window",
		"//common/proto:cc_route_proto",
        "//homework6/map:map_lib",
		"//homework5/geometry:geometry",
        "@gflags",
        "@glog",
    ],
)


