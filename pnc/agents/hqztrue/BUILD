package(default_visibility = ["//visibility:public"])

cc_library(
    name = "hqztrue_agent",
    hdrs = ["hqztrue_agent.h"],
    deps = [
        "//pnc/simulation:vehicle_agent",
        "//pnc/simulation:vehicle_agent_factory",
        "//pnc/agents/hqztrue/route:find_route",
        "//common/proto:cc_agent_status_proto",
        "//common/proto:cc_control_proto",
        "//common/proto:cc_simulation_proto",
        "//common/utils/math",
        "//common/utils/math/transform",
        "@glog",
    ],
)
