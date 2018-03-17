package(default_visibility = ["//visibility:public"])

# Adapted from the toolchain BUILD file shipped with bazel.

# This is the entry point for --crosstool_top.  Toolchains are found
# by lopping off the name of --crosstool_top and searching for
# "cc-compiler-${CPU}" in this BUILD file, where CPU is the target CPU
# specified in --cpu.
#
# This file group should include
#   * all cc_toolchain targets supported
#   * all file groups that said cc_toolchain might refer to,
# including the default_grte_top setting in the CROSSTOOL
# protobuf.
alias(
    name = "toolchain",
    actual = "//external:cc_toolchain",
)

config_setting(
    name = "dbg_enabled",
    values = {
        "compilation_mode": "dbg",
    },
)

config_setting(
    name = "fastbuild_enabled",
    values = {
        "compilation_mode": "fastbuild",
    },
)

config_setting(
    name = "opt_enabled",
    values = {
        "compilation_mode": "opt",
    },
)

config_setting(
    name = "asan_enabled",
    values = {
        'copt': '-fsanitize=address'
    }
)

config_setting(
    name = "cuda_enabled",
    define_values = {
        "enable_cuda": "true",
    },
)

config_setting(
    name = "remote_cache_disabled",
    define_values = {
        "enable_remote_cache": "false",
    }
)

# Hardcoded toolchain, legacy behaviour.
cc_toolchain_suite(
    name = "default",
    toolchains = {
        "local|compiler": ":cc-compiler-local",
    },
)

cc_toolchain(
    name = "cc-compiler-local",
    all_files = ":empty",
    compiler_files = ":empty",
    cpu = "local",
    dwp_files = ":empty",
    dynamic_runtime_libs = [":empty"],
    linker_files = ":empty",
    objcopy_files = ":empty",
    static_runtime_libs = [":empty"],
    strip_files = ":empty",
    supports_param_files = 1,
)

filegroup(name = "toolchain_category")

filegroup(
    name = "empty",
    srcs = [],
)
