package(default_visibility = ["//visibility:public"])

cc_library(
    name = "rotation",
    srcs = ["rotation.cc"],
    hdrs = ["rotation.h"],
    deps = [
        "@eigen//:eigen",
    ],
)

cc_test(
    name = "rotation_test",
    srcs = ["rotation_test.cc"],
    deps = [
        ":rotation",
        "//utils/testing:test_main",
    ],
)
