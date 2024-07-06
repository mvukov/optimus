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
    sha256 = "044b148b111e17cee61d8c3aaeed42069f7325460382340afca4919f8265094c",
    strip_prefix = "bazel-compile-commands-extractor-1e08f8e0507b6b6b1f4416a9a22cf5c28beaba93",
    url = "https://github.com/hedronvision/bazel-compile-commands-extractor/archive/1e08f8e0507b6b6b1f4416a9a22cf5c28beaba93.tar.gz",
)

load("@hedron_compile_commands//:workspace_setup.bzl", "hedron_compile_commands_setup")

hedron_compile_commands_setup()

load("@hedron_compile_commands//:workspace_setup_transitive.bzl", "hedron_compile_commands_setup_transitive")

hedron_compile_commands_setup_transitive()

load("@hedron_compile_commands//:workspace_setup_transitive_transitive.bzl", "hedron_compile_commands_setup_transitive_transitive")

hedron_compile_commands_setup_transitive_transitive()

load("@hedron_compile_commands//:workspace_setup_transitive_transitive_transitive.bzl", "hedron_compile_commands_setup_transitive_transitive_transitive")

hedron_compile_commands_setup_transitive_transitive_transitive()
