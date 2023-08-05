workspace(name = "com_github_mvukov_optimus")

load("//repositories:repositories.bzl", "optimus_repositories")

optimus_repositories()

load("//repositories:deps.bzl", "optimus_deps")

optimus_deps()

load("@rules_python//python:repositories.bzl", "python_register_toolchains")

python_register_toolchains(
    name = "optimus_python",
    python_version = "3.8.13",
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
    sha256 = "8c00618fa6f00f24376547439f92ebc1e11f45a33a09d9f0a492c8e0d83cbfde",
    strip_prefix = "bazel-compile-commands-extractor-aaa2fda89b40e89afa2130b41b56eb839a769be5",
    url = "https://github.com/hedronvision/bazel-compile-commands-extractor/archive/aaa2fda89b40e89afa2130b41b56eb839a769be5.tar.gz",
)

load("@hedron_compile_commands//:workspace_setup.bzl", "hedron_compile_commands_setup")

hedron_compile_commands_setup()
