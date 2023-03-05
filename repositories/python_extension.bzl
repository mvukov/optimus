load("@rules_cc//cc:defs.bzl", "cc_binary")
load("@rules_python//python:defs.bzl", "py_library")

def py_extension(name, **kwargs):
    """Defines a Python C/C++ extension.

    Args:
        name: The extension name.
        kwargs: C++ options.
    """
    name_with_extension = name + ".so"

    copts = kwargs.pop("copts", [])
    linkopts = kwargs.pop("linkopts", [])

    linkshared = kwargs.pop("linkshared", 1)
    if not linkshared:
        fail("Python extension libraries must be shared libraries!")

    deps = kwargs.pop("deps", [])
    tags = kwargs.pop("tags", [])
    target_compatible_with = kwargs.pop("target_compatible_with", [])
    visibility = kwargs.pop("visibility", None)
    cc_binary(
        name = name_with_extension,
        copts = ["-fvisibility=hidden", "-fsized-deallocation"] + copts,
        linkopts = ["-fvisibility=hidden"] + linkopts,
        linkshared = linkshared,
        deps = ["@optimus_python//:python_headers"] + deps,
        tags = ["manual"],
        **kwargs
    )

    py_library(
        name = name,
        data = [":{}".format(name_with_extension)],
        tags = tags,
        target_compatible_with = target_compatible_with,
        visibility = visibility,
    )

def pybind11_extension(name, **kwargs):
    """Defines a pybind11 C++ extension for Python.

    Args:
        name: The extension name.
        kwargs: C++ options.
    """
    deps = kwargs.pop("deps", []) + ["@pybind11//:pybind11"]
    kwargs.update(deps = deps)
    py_extension(name, **kwargs)
