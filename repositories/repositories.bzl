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
        sha256 = "f700c75859a827a2e3e3ba4c9c0ec2d796e191bf0438ac3fee0b7851d83a3d4c",
        strip_prefix = "rules_python-2.3.3",
        url = "https://github.com/bazelbuild/rules_python/archive/refs/tags/2.3.3.tar.gz",
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
        sha256 = "9631341c82bac4a288bef951f8b26b41f69021794184ece969f8473977eaa340",
        strip_prefix = "benchmark-1.9.5",
        urls = ["https://github.com/google/benchmark/archive/refs/tags/v1.9.5.tar.gz"],
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
        sha256 = "9a955b1b49a4410088a2e0ee2a9c057c3c907d0c1d75454144cb980aca0ba515",
        strip_prefix = "stb-2c980bb59875b0d32144a71867fbdebb2f77cd20",
        urls = ["https://github.com/nothings/stb/archive/2c980bb59875b0d32144a71867fbdebb2f77cd20.tar.gz"],
        build_file = "//repositories:stb.BUILD.bazel",
    )
