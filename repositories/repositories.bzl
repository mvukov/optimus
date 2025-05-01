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
        sha256 = "9e23d9b584ef651a07f81055bafaec6aa7f5026b7ff0a30b73e9c78880a829cc",
        strip_prefix = "rules_boost-ec53a3b510be6a9fb41508cf884fe62d4098c5eb",
        urls = ["https://github.com/nelhage/rules_boost/archive/ec53a3b510be6a9fb41508cf884fe62d4098c5eb.zip"],
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
        sha256 = "409075176168dc46bbb81b74c1b4b6900385b5d16bfc181d678afb060d928bd3",
        strip_prefix = "benchmark-1.9.2",
        urls = ["https://github.com/google/benchmark/archive/refs/tags/v1.9.2.tar.gz"],
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
        sha256 = "4d05c96640ae3a8cbdafdad8d344b50ce610802f78aee80154acdb8e266282e0",
        strip_prefix = "stb-f0569113c93ad095470c54bf34a17b36646bbbb5",
        urls = ["https://github.com/nothings/stb/archive/f0569113c93ad095470c54bf34a17b36646bbbb5.tar.gz"],
        build_file = "//repositories:stb.BUILD.bazel",
    )
