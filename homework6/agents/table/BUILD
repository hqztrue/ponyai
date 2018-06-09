package(default_visibility = ["//visibility:public"])

cc_library(
    name = "table_agent",
    hdrs = ["table_agent.h"],
    deps = [
        "//homework6/simulation:vehicle_agent",
        "//homework6/simulation:vehicle_agent_factory",
		"//homework6/route:find_route",
        "//common/proto:cc_agent_status_proto",
        "//common/proto:cc_control_proto",
        "//common/proto:cc_simulation_proto",
        "@glog",
    ],
)
