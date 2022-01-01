workspace(name = "com_github_mvukov_optimus")

load("//repositories:repositories.bzl", "optimus_repositories")

optimus_repositories()

load("//repositories:deps.bzl", "optimus_deps")

optimus_deps()

# NOTE: Down here are listed dependencies needed for tests and benchmarks.

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

http_archive(
    name = "rules_python",
    sha256 = "778197e26c5fbeb07ac2a2c5ae405b30f6cb7ad1f5510ea6fdac03bded96cc6f",
    urls = ["https://github.com/bazelbuild/rules_python/releases/download/0.2.0/rules_python-0.2.0.tar.gz"],
)

http_archive(
    name = "com_google_googletest",
    urls = ["https://github.com/google/googletest/archive/refs/tags/release-1.11.0.tar.gz"],
    sha256 = "b4870bf121ff7795ba20d20bcdd8627b8e088f2d1dab299a031c1034eddc93d5",
    strip_prefix = "googletest-release-1.11.0",
)

http_archive(
    name = "com_github_google_benchmark",
    urls = ["https://github.com/google/benchmark/archive/refs/tags/v1.5.5.tar.gz"],
    sha256 = "3bff5f237c317ddfd8d5a9b96b3eede7c0802e799db520d38ce756a2a46a18a0",
    strip_prefix = "benchmark-1.5.5",
)

load("@rules_python//python:pip.bzl", "pip_parse")

_PYTHON_INTERPRETER = "python3.8"

pip_parse(
    name = "optimus_pip_deps",
    python_interpreter = _PYTHON_INTERPRETER,
    requirements_lock = "//:requirements_lock.txt",
)

load(
    "@optimus_pip_deps//:requirements.bzl",
    install_optimus_pip_deps = "install_deps",
)

install_optimus_pip_deps()
