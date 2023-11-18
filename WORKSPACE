workspace(name = "com_github_mvukov_optimus")

load("//repositories:repositories.bzl", "optimus_repositories")

optimus_repositories()

load("//repositories:deps.bzl", "optimus_deps")

optimus_deps()

load("@rules_python//python:repositories.bzl", "py_repositories", "python_register_toolchains")

py_repositories()

python_register_toolchains(
    name = "optimus_python",
    python_version = "3.10",
)

load("@optimus_python//:defs.bzl", python_interpreter_target = "interpreter")
load("@rules_python//python:pip.bzl", "pip_parse")

pip_parse(
    name = "optimus_pip_deps",
    python_interpreter_target = python_interpreter_target,
    requirements_lock = "//:requirements_lock.txt",
)

load(
    "@optimus_pip_deps//:requirements.bzl",
    install_optimus_pip_deps = "install_deps",
)

install_optimus_pip_deps()

# Internal dependencies.

load("//repositories:test_data.bzl", "test_data")

test_data()

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

http_archive(
    name = "hedron_compile_commands",
    sha256 = "05f7fb324290c147ed29361a6a6fef7459c61816084fc07b3676a9639f4fcab7",
    strip_prefix = "bazel-compile-commands-extractor-eca42c63700fccdc49cf58177e0a96f0f6075a68",
    url = "https://github.com/hedronvision/bazel-compile-commands-extractor/archive/eca42c63700fccdc49cf58177e0a96f0f6075a68.tar.gz",
)

load("@hedron_compile_commands//:workspace_setup.bzl", "hedron_compile_commands_setup")

hedron_compile_commands_setup()
