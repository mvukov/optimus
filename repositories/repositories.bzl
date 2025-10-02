"""Handles import of external/third-party repositories.
"""

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:utils.bzl", "maybe")

def optimus_repositories():
    maybe(
        http_archive,
        name = "bazel_skylib",
        urls = ["https://github.com/bazelbuild/bazel-skylib/releases/download/1.8.2/bazel-skylib-1.8.2.tar.gz"],
        sha256 = "6e78f0e57de26801f6f564fa7c4a48dc8b36873e416257a92bbb0937eeac8446",
    )

    maybe(
        http_archive,
        name = "eigen",
        build_file = "@com_github_mvukov_optimus//repositories:eigen.BUILD.bazel",
        sha256 = "8586084f71f9bde545ee7fa6d00288b264a2b7ac3607b974e54d13e7162c1c72",
        strip_prefix = "eigen-3.4.0",
        urls = ["https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz"],
    )

    maybe(
        http_archive,
        name = "com_github_mvukov_qpoases_embedded",
        sha256 = "8834df1bbd12c21a69d8f2de6e6a2066e8ebd91f91371f7eeb72fe415934cb34",
        strip_prefix = "qpoases_embedded-0.1.0",
        urls = ["https://github.com/mvukov/qpoases_embedded/archive/refs/tags/v0.1.0.tar.gz"],
    )

    maybe(
        http_archive,
        name = "com_github_nelhage_rules_boost",
        sha256 = "11321ea98ac39cc122e86a12e737e953ff40f9ce349acbfc851bf9b189e7fab4",
        strip_prefix = "rules_boost-2fa8365555016fd48d8c8042d7adb5bbc8841573",
        urls = ["https://github.com/nelhage/rules_boost/archive/2fa8365555016fd48d8c8042d7adb5bbc8841573.zip"],
    )

    maybe(
        http_archive,
        name = "rules_python",
        sha256 = "690e0141724abb568267e003c7b6d9a54925df40c275a870a4d934161dc9dd53",
        strip_prefix = "rules_python-0.40.0",
        url = "https://github.com/bazelbuild/rules_python/archive/refs/tags/0.40.0.tar.gz",
    )

    maybe(
        http_archive,
        name = "com_google_googletest",
        sha256 = "65fab701d9829d38cb77c14acdc431d2108bfdbf8979e40eb8ae567edf10b27c",
        strip_prefix = "googletest-1.17.0",
        urls = ["https://github.com/google/googletest/archive/refs/tags/v1.17.0.tar.gz"],
    )

    maybe(
        http_archive,
        name = "com_github_google_benchmark",
        sha256 = "b334658edd35efcf06a99d9be21e4e93e092bd5f95074c1673d5c8705d95c104",
        strip_prefix = "benchmark-1.9.4",
        urls = ["https://github.com/google/benchmark/archive/refs/tags/v1.9.4.tar.gz"],
    )

    maybe(
        http_archive,
        name = "pybind11",
        build_file = "//repositories:pybind11.BUILD.bazel",
        sha256 = "453b1a3e2b266c3ae9da872411cadb6d693ac18063bd73226d96cfb7015a200c",
        strip_prefix = "pybind11-3.0.0",
        url = "https://github.com/pybind/pybind11/archive/v3.0.0.tar.gz",
    )

    maybe(
        http_archive,
        name = "stb",
        sha256 = "3c45e9ab123933274ad782e19fc4029444bfe26e0a5eea483c30a82b880b8a42",
        strip_prefix = "stb-fede005abaf93d9d7f3a679d1999b2db341b360f",
        urls = ["https://github.com/nothings/stb/archive/fede005abaf93d9d7f3a679d1999b2db341b360f.tar.gz"],
        build_file = "//repositories:stb.BUILD.bazel",
    )
