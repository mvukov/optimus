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
        sha256 = "60fafa69ad41e6d76111d6f0b498415be5fddb85a383ab94926a2c6e53b73c2d",
        strip_prefix = "rules_boost-70faf49298db399db6d86d55e29fbec528779f4c",
        urls = ["https://github.com/nelhage/rules_boost/archive/70faf49298db399db6d86d55e29fbec528779f4c.zip"],
    )

    maybe(
        http_archive,
        name = "rules_python",
        sha256 = "ca2671529884e3ecb5b79d6a5608c7373a82078c3553b1fa53206e6b9dddab34",
        strip_prefix = "rules_python-0.38.0",
        url = "https://github.com/bazelbuild/rules_python/archive/refs/tags/0.38.0.tar.gz",
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
        sha256 = "32131c08ee31eeff2c8968d7e874f3cb648034377dfc32a4c377fa8796d84981",
        strip_prefix = "benchmark-1.9.1",
        urls = ["https://github.com/google/benchmark/archive/refs/tags/v1.9.1.tar.gz"],
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
        sha256 = "cfeab9f800961882d6d22ddf36e965523b33002f4f937de08321304c9ba72af3",
        strip_prefix = "stb-5c205738c191bcb0abc65c4febfa9bd25ff35234",
        urls = ["https://github.com/nothings/stb/archive/5c205738c191bcb0abc65c4febfa9bd25ff35234.tar.gz"],
        build_file = "//repositories:stb.BUILD.bazel",
    )
