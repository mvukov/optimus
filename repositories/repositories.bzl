"""Handles import of external/third-party repositories.
"""

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:utils.bzl", "maybe")

def optimus_repositories():
    maybe(
        http_archive,
        name = "bazel_skylib",
        urls = ["https://github.com/bazelbuild/bazel-skylib/releases/download/1.7.1/bazel-skylib-1.7.1.tar.gz"],
        sha256 = "bc283cdfcd526a52c3201279cda4bc298652efa898b10b4db0837dc51652756f",
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
        sha256 = "fb8ff15cfc71a8abb0bca369db2646ddd39b30454fb3ab54faf0e936a86972ba",
        strip_prefix = "rules_boost-e9bd4c17c976fa4ee0f3889d497e1d333f1f95d5",
        urls = ["https://github.com/nelhage/rules_boost/archive/e9bd4c17c976fa4ee0f3889d497e1d333f1f95d5.zip"],
    )

    maybe(
        http_archive,
        name = "rules_python",
        sha256 = "ca77768989a7f311186a29747e3e95c936a41dffac779aff6b443db22290d913",
        strip_prefix = "rules_python-0.36.0",
        url = "https://github.com/bazelbuild/rules_python/archive/refs/tags/0.36.0.tar.gz",
    )

    maybe(
        http_archive,
        name = "com_google_googletest",
        sha256 = "7b42b4d6ed48810c5362c265a17faebe90dc2373c885e5216439d37927f02926",
        strip_prefix = "googletest-1.15.2",
        urls = ["https://github.com/google/googletest/archive/refs/tags/v1.15.2.tar.gz"],
    )

    maybe(
        http_archive,
        name = "com_github_google_benchmark",
        sha256 = "35a77f46cc782b16fac8d3b107fbfbb37dcd645f7c28eee19f3b8e0758b48994",
        strip_prefix = "benchmark-1.9.0",
        urls = ["https://github.com/google/benchmark/archive/refs/tags/v1.9.0.tar.gz"],
    )

    maybe(
        http_archive,
        name = "pybind11",
        build_file = "//repositories:pybind11.BUILD.bazel",
        sha256 = "e08cb87f4773da97fa7b5f035de8763abc656d87d5773e62f6da0587d1f0ec20",
        strip_prefix = "pybind11-2.13.6",
        url = "https://github.com/pybind/pybind11/archive/v2.13.6.tar.gz",
    )

    maybe(
        http_archive,
        name = "stb",
        sha256 = "bc6ccf08bec08fea8ef423c7117dca06d2f62d2b27c5485f6865584b533fa7fa",
        strip_prefix = "stb-f75e8d1cad7d90d72ef7a4661f1b994ef78b4e31",
        urls = ["https://github.com/nothings/stb/archive/f75e8d1cad7d90d72ef7a4661f1b994ef78b4e31.tar.gz"],
        build_file = "//repositories:stb.BUILD.bazel",
    )
