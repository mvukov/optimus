"""Handles import of external/third-party repositories.
"""

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:utils.bzl", "maybe")

def optimus_repositories():
    maybe(
        http_archive,
        name = "bazel_skylib",
        urls = ["https://github.com/bazelbuild/bazel-skylib/releases/download/1.4.2/bazel-skylib-1.4.2.tar.gz"],
        sha256 = "66ffd9315665bfaafc96b52278f57c7e2dd09f5ede279ea6d39b2be471e7e3aa",
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
        sha256 = "0b4c4f709768108a6e0bec1727e32e646292de6b47bc2e24fc2b51cd8271fd05",
        strip_prefix = "rules_boost-0ba0fd7d7f812deb9cac879ee6cbf237276db96d",
        urls = ["https://github.com/nelhage/rules_boost/archive/0ba0fd7d7f812deb9cac879ee6cbf237276db96d.zip"],
    )

    maybe(
        http_archive,
        name = "rules_python",
        sha256 = "84aec9e21cc56fbc7f1335035a71c850d1b9b5cc6ff497306f84cced9a769841",
        strip_prefix = "rules_python-0.23.1",
        url = "https://github.com/bazelbuild/rules_python/archive/refs/tags/0.23.1.tar.gz",
    )

    maybe(
        http_archive,
        name = "com_google_googletest",
        sha256 = "b4870bf121ff7795ba20d20bcdd8627b8e088f2d1dab299a031c1034eddc93d5",
        strip_prefix = "googletest-release-1.11.0",
        urls = ["https://github.com/google/googletest/archive/refs/tags/release-1.11.0.tar.gz"],
    )

    maybe(
        http_archive,
        name = "com_github_google_benchmark",
        sha256 = "ea2e94c24ddf6594d15c711c06ccd4486434d9cf3eca954e2af8a20c88f9f172",
        strip_prefix = "benchmark-1.8.0",
        urls = ["https://github.com/google/benchmark/archive/refs/tags/v1.8.0.tar.gz"],
    )

    maybe(
        http_archive,
        name = "pybind11",
        build_file = "//repositories:pybind11.BUILD.bazel",
        sha256 = "832e2f309c57da9c1e6d4542dedd34b24e4192ecb4d62f6f4866a737454c9970",
        strip_prefix = "pybind11-2.10.4",
        url = "https://github.com/pybind/pybind11/archive/v2.10.4.tar.gz",
    )
