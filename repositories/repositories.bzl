"""Handles import of external/third-party repositories.
"""

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:utils.bzl", "maybe")

def optimus_repositories():
    maybe(
        http_archive,
        name = "bazel_skylib",
        urls = ["https://github.com/bazelbuild/bazel-skylib/releases/download/1.7.0/bazel-skylib-1.7.0.tar.gz"],
        sha256 = "d00f1389ee20b60018e92644e0948e16e350a7707219e7a390fb0a99b6ec9262",
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
        sha256 = "ea8541ee324155e25ec085c70fc8bc92164522ec3e1e458c262fc83d77c381a7",
        strip_prefix = "rules_boost-5caf74fbeb42ec1f414c5c0f7824953370a9490d",
        urls = ["https://github.com/nelhage/rules_boost/archive/5caf74fbeb42ec1f414c5c0f7824953370a9490d.zip"],
    )

    maybe(
        http_archive,
        name = "rules_python",
        sha256 = "4912ced70dc1a2a8e4b86cec233b192ca053e82bc72d877b98e126156e8f228d",
        strip_prefix = "rules_python-0.32.2",
        url = "https://github.com/bazelbuild/rules_python/archive/refs/tags/0.32.2.tar.gz",
    )

    maybe(
        http_archive,
        name = "com_google_googletest",
        sha256 = "8ad598c73ad796e0d8280b082cebd82a630d73e73cd3c70057938a6501bba5d7",
        strip_prefix = "googletest-1.14.0",
        urls = ["https://github.com/google/googletest/archive/refs/tags/v1.14.0.tar.gz"],
    )

    maybe(
        http_archive,
        name = "com_github_google_benchmark",
        sha256 = "3e7059b6b11fb1bbe28e33e02519398ca94c1818874ebed18e504dc6f709be45",
        strip_prefix = "benchmark-1.8.4",
        urls = ["https://github.com/google/benchmark/archive/refs/tags/v1.8.4.tar.gz"],
    )

    maybe(
        http_archive,
        name = "pybind11",
        build_file = "//repositories:pybind11.BUILD.bazel",
        sha256 = "bf8f242abd1abcd375d516a7067490fb71abd79519a282d22b6e4d19282185a7",
        strip_prefix = "pybind11-2.12.0",
        url = "https://github.com/pybind/pybind11/archive/v2.12.0.tar.gz",
    )

    maybe(
        http_archive,
        name = "stb",
        sha256 = "ee084ead38afd9e974e0e9777949b1246dea07759e0e3a8f421123ea9a3c69e0",
        strip_prefix = "stb-449758bd74ce14b7d0ba9b24a3dbc4386702a0e0",
        urls = ["https://github.com/nothings/stb/archive/449758bd74ce14b7d0ba9b24a3dbc4386702a0e0.tar.gz"],
        build_file = "//repositories:stb.BUILD.bazel",
    )
