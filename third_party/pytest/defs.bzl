# NOTE: adapted from https://dev.to/davidb31/experimentations-on-bazel-python-3-linter-pytest-49oh
# NOTE: Adapted from https://gist.github.com/betaboon/c1dd785b5ba468b4df4e382eafff969a

load("@optimus_pip_deps//:requirements.bzl", "requirement")
load("@rules_python//python:defs.bzl", "py_test")

def py_pytest_test(name, srcs, deps = [], args = [], **kwargs):
    py_test(
        name = name,
        srcs = [
            "//third_party/pytest:pytest_wrapper.py",
        ] + srcs,
        main = "//third_party/pytest:pytest_wrapper.py",
        args = [
            "-ra",
            "-vv",
        ] + args + ["$(location :%s)" % x for x in srcs],
        python_version = "PY3",
        srcs_version = "PY3",
        deps = deps + [
            requirement("coverage"),
            requirement("pytest"),
            requirement("pytest-cov"),
        ],
        **kwargs
    )
