"""Handles import of external/third-party repositories.
"""

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:utils.bzl", "maybe")

def optimus_repositories():
    maybe(
        http_archive,
        name = "bazel_skylib",
        urls = ["https://github.com/bazelbuild/bazel-skylib/releases/download/1.9.2/bazel-skylib-1.9.2.tar.gz"],
        sha256 = "37cdfbc6faefea94f7b37760a305c98c08981116c2bc9e821e3b423221fad8c8",
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
        sha256 = "690e0141724abb568267e003c7b6d9a54925df40c275a870a4d934161dc9dd53",
        strip_prefix = "rules_python-0.40.0",
        url = "https://github.com/bazelbuild/rules_python/archive/refs/tags/0.40.0.tar.gz",
    )

    maybe(
        http_archive,
        name = "com_google_googletest",
        sha256 = "6e3191c1455468b3fc35a417fb565c1c5071aee1b7e7f85e30cf48a98d37d8b5",
        strip_prefix = "googletest-1.18.0",
        urls = ["https://github.com/google/googletest/archive/refs/tags/v1.18.0.tar.gz"],
    )

    maybe(
        http_archive,
        name = "com_github_google_benchmark",
        sha256 = "acc5c7c0fbed855c4aaa2d7c3c36fb82b0c15c28f4e151e3f679c2fc0aa404af",
        strip_prefix = "benchmark-1.10.0",
        urls = ["https://github.com/google/benchmark/archive/refs/tags/v1.10.0.tar.gz"],
    )

    maybe(
        http_archive,
        name = "pybind11",
        build_file = "//repositories:pybind11.BUILD.bazel",
        sha256 = "ef712655692a2e9bf7bb7874c022564a45f91d847ddee987e720cd9e28849665",
        strip_prefix = "pybind11-3.1.0",
        url = "https://github.com/pybind/pybind11/archive/v3.1.0.tar.gz",
    )

    maybe(
        http_archive,
        name = "stb",
        sha256 = "a6c31950b3bdf05eb92258768ba2a11bd44449490574b1aac67028452288c40d",
        strip_prefix = "stb-003501f6d7a8668ce8b6d875e34dd97ad0d5d5b5",
        urls = ["https://github.com/nothings/stb/archive/003501f6d7a8668ce8b6d875e34dd97ad0d5d5b5.tar.gz"],
        build_file = "//repositories:stb.BUILD.bazel",
    )
