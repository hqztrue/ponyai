package(default_visibility = ["//visibility:public"])

cc_library(
    name = "find_route",
	srcs = ["find_route.cc"],
    hdrs = ["find_route.h"],
    deps = [
        "//common/proto:cc_map_proto",
        "//common/utils/file",
		#"//pnc/display:main_window",
		"//common/proto:cc_route_proto",
        "//pnc/map:map_lib",
		"//pnc/agents/hqztrue/geometry:geometry",
        "@gflags",
        "@glog",
    ],
)


