workspace(name = "ponyai_public_course")

# gtest
new_http_archive(
    name = "googletest",
    urls = ["https://github.com/google/googletest/archive/release-1.8.0.tar.gz"],
    strip_prefix = "googletest-release-1.8.0",
    build_file = "utils/bazel/googletest.BUILD",
    sha256 = "58a6f4277ca2bc8565222b3bbd58a177609e9c488e8a72649359ba51450db7d8",
)

# glog
new_http_archive(
    name = "glog",
    urls = ["https://github.com/google/glog/archive/v0.3.5.tar.gz"],
    build_file = "utils/bazel/glog.BUILD",
    strip_prefix = "glog-0.3.5",
    sha256 = "7580e408a2c0b5a89ca214739978ce6ff480b5e7d8d7698a2aa92fadc484d1e0",
)

# gflags
http_archive(
    name = "gflags",
    urls = ["https://github.com/gflags/gflags/archive/v2.2.1.tar.gz"],
    strip_prefix = "gflags-2.2.1",
    sha256 = "ae27cdbcd6a2f935baa78e4f21f675649271634c092b1be01469440495609d0e",
)

# protobuf
http_archive(
    name = "com_google_protobuf",
    urls = ["https://github.com/google/protobuf/archive/v3.4.1.tar.gz"],
    strip_prefix = "protobuf-3.4.1",
    sha256 = "8e0236242106e680b4f9f576cc44b8cd711e948b20a9fc07769b0a20ceab9cc4",
)


