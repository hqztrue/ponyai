package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "map_visualizer_main",
    srcs = ["map_visualizer_main.cc"],
    deps = [
        "//common/utils/file",
        "//homework5/display:main_window",
        "//homework5/map:map_lib",
		"//homework5/route:find_route",
        "@gflags//:gflags",
        "@glog",
    ],
)
