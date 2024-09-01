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
        sha256 = "2ff3ab2b6205dc52f9fa975d5e6b795e6547f66653d3ecdc6bf48ca7ffd8fbee",
        strip_prefix = "rules_boost-58be4e7e851d19e9ba14ced7bdba6fc8895af1d3",
        urls = ["https://github.com/nelhage/rules_boost/archive/58be4e7e851d19e9ba14ced7bdba6fc8895af1d3.zip"],
    )

    maybe(
        http_archive,
        name = "rules_python",
        sha256 = "be04b635c7be4604be1ef20542e9870af3c49778ce841ee2d92fcb42f9d9516a",
        strip_prefix = "rules_python-0.35.0",
        url = "https://github.com/bazelbuild/rules_python/archive/refs/tags/0.35.0.tar.gz",
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
        sha256 = "b1e209c42b3a9ed74da3e0b25a4f4cd478d89d5efbb48f04b277df427faf6252",
        strip_prefix = "pybind11-2.13.5",
        url = "https://github.com/pybind/pybind11/archive/v2.13.5.tar.gz",
    )

    maybe(
        http_archive,
        name = "stb",
        sha256 = "bc6ccf08bec08fea8ef423c7117dca06d2f62d2b27c5485f6865584b533fa7fa",
        strip_prefix = "stb-f75e8d1cad7d90d72ef7a4661f1b994ef78b4e31",
        urls = ["https://github.com/nothings/stb/archive/f75e8d1cad7d90d72ef7a4661f1b994ef78b4e31.tar.gz"],
        build_file = "//repositories:stb.BUILD.bazel",
    )
