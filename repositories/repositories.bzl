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
        sha256 = "ca61af8fcec97efefd7fa1b892f3b02ed46f0700a04c4a8ab8bae36f4868317b",
        strip_prefix = "rules_boost-5cdb3c83807d1036bebcc60acf87333e5b3cc856",
        urls = ["https://github.com/nelhage/rules_boost/archive/5cdb3c83807d1036bebcc60acf87333e5b3cc856.zip"],
    )

    maybe(
        http_archive,
        name = "rules_python",
        sha256 = "5bcfa3852444d084b1d3262714cec151b797648d4d444ea9895c7c7ed79cd715",
        strip_prefix = "rules_python-0.33.1",
        url = "https://github.com/bazelbuild/rules_python/archive/refs/tags/0.33.1.tar.gz",
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
        sha256 = "bc6ccf08bec08fea8ef423c7117dca06d2f62d2b27c5485f6865584b533fa7fa",
        strip_prefix = "stb-f75e8d1cad7d90d72ef7a4661f1b994ef78b4e31",
        urls = ["https://github.com/nothings/stb/archive/f75e8d1cad7d90d72ef7a4661f1b994ef78b4e31.tar.gz"],
        build_file = "//repositories:stb.BUILD.bazel",
    )
