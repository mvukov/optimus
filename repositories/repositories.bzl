"""Handles import of external/third-party repositories.
"""

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:utils.bzl", "maybe")

def optimus_repositories():
    maybe(
        http_archive,
        name = "bazel_skylib",
        urls = ["https://github.com/bazelbuild/bazel-skylib/releases/download/1.8.0/bazel-skylib-1.8.0.tar.gz"],
        sha256 = "fa01292859726603e3cd3a0f3f29625e68f4d2b165647c72908045027473e933",
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
        sha256 = "95ea0b2f49377c738588677756a8b2c0da174e50e77c92d5fe2cc5d567687c10",
        strip_prefix = "rules_boost-2a7a2ad203e469276540cb7d260e8bd46f6cc030",
        urls = ["https://github.com/nelhage/rules_boost/archive/2a7a2ad203e469276540cb7d260e8bd46f6cc030.zip"],
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
        sha256 = "e08cb87f4773da97fa7b5f035de8763abc656d87d5773e62f6da0587d1f0ec20",
        strip_prefix = "pybind11-2.13.6",
        url = "https://github.com/pybind/pybind11/archive/v2.13.6.tar.gz",
    )

    maybe(
        http_archive,
        name = "stb",
        sha256 = "be1a5dad9ac38f42ef8abf00bd7db776101661b77603f832fcc0bae843d82846",
        strip_prefix = "stb-f58f558c120e9b32c217290b80bad1a0729fbb2c",
        urls = ["https://github.com/nothings/stb/archive/f58f558c120e9b32c217290b80bad1a0729fbb2c.tar.gz"],
        build_file = "//repositories:stb.BUILD.bazel",
    )
