"""Handles import of external/third-party repositories.
"""

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:utils.bzl", "maybe")

def optimus_repositories():
    maybe(
        http_archive,
        name = "bazel_skylib",
        urls = ["https://github.com/bazelbuild/bazel-skylib/releases/download/1.9.0/bazel-skylib-1.9.0.tar.gz"],
        sha256 = "3b5b49006181f5f8ff626ef8ddceaa95e9bb8ad294f7b5d7b11ea9f7ddaf8c59",
    )

    maybe(
        http_archive,
        name = "eigen",
        build_file = "@com_github_mvukov_optimus//repositories:eigen.BUILD.bazel",
        sha256 = "e9c326dc8c05cd1e044c71f30f1b2e34a6161a3b6ecf445d56b53ff1669e3dec",
        strip_prefix = "eigen-5.0.1",
        urls = ["https://gitlab.com/libeigen/eigen/-/archive/5.0.1/eigen-5.0.1.tar.gz"],
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
        sha256 = "e11d2e1efce1589e5bdfa93986712c74fc7467a0f093143d489d2ef5ebb1ed2a",
        strip_prefix = "rules_python-2.2.0",
        url = "https://github.com/bazelbuild/rules_python/archive/refs/tags/2.2.0.tar.gz",
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
        sha256 = "9631341c82bac4a288bef951f8b26b41f69021794184ece969f8473977eaa340",
        strip_prefix = "benchmark-1.9.5",
        urls = ["https://github.com/google/benchmark/archive/refs/tags/v1.9.5.tar.gz"],
    )

    maybe(
        http_archive,
        name = "pybind11",
        build_file = "//repositories:pybind11.BUILD.bazel",
        sha256 = "74b6a2c2b4573a400cafb6ecbf60c98df300cd3d0041296b913d02b2cbbb2676",
        strip_prefix = "pybind11-3.0.4",
        url = "https://github.com/pybind/pybind11/archive/v3.0.4.tar.gz",
    )

    maybe(
        http_archive,
        name = "stb",
        sha256 = "e4e3bba9c572a4a4148373a914d88ea0f0d11de8cc2c66739926e7eca0223319",
        strip_prefix = "stb-31c1ad37456438565541f4919958214b6e762fb4",
        urls = ["https://github.com/nothings/stb/archive/31c1ad37456438565541f4919958214b6e762fb4.tar.gz"],
        build_file = "//repositories:stb.BUILD.bazel",
    )
