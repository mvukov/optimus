""" Defines py_env macro.
"""

PyImportsAspectInfo = provider(
    "Provides imports",
    fields = [
        "imports",
    ],
)

def _py_imports_aspect_impl(target, ctx):
    imports = []
    if PyInfo in target:
        imports.append(target[PyInfo].imports)

    if hasattr(ctx.rule.attr, "deps"):
        imports.extend([
            dep[PyImportsAspectInfo].imports
            for dep in ctx.rule.attr.deps
        ])

    return [
        PyImportsAspectInfo(
            imports = depset(
                transitive = imports,
            ),
        ),
    ]

py_imports_aspect = aspect(
    implementation = _py_imports_aspect_impl,
    attr_aspects = ["deps"],
    provides = [PyImportsAspectInfo],
)

def py_env_rule_impl(ctx):
    imports = depset(
        transitive = [
            dep[PyImportsAspectInfo].imports
            for dep in ctx.attr.deps
        ],
    )

    out_name = ctx.label.name + ".sh"
    runfiles_dir = out_name + ".runfiles"
    out_paths = []
    for import_path in imports.to_list():
        out_paths.append("\\${{PYTHONPATH_ROOT}}/{}/{}".format(runfiles_dir, import_path))

    # Output the .env file to the workspace root.
    script_content = """#!/bin/bash
__output="$BUILD_WORKSPACE_DIRECTORY/.env"
__root=$BUILD_WORKSPACE_DIRECTORY/bazel-bin/{package_name}
echo "PYTHONPATH_ROOT=$__root\nPYTHONPATH={python_path}:$BUILD_WORKSPACE_DIRECTORY" > "$__output"
echo "Wrote .env file to $__output"
""".format(
        python_path = ":".join(out_paths),
        package_name = ctx.attr.package_name,
    )

    script = ctx.actions.declare_file(out_name)
    ctx.actions.write(script, script_content, is_executable = True)

    runfiles = ctx.runfiles()
    for dep in ctx.attr.deps:
        runfiles = runfiles.merge(dep[DefaultInfo].default_runfiles)

    return [
        DefaultInfo(
            executable = script,
            runfiles = runfiles,
        ),
    ]

py_env_rule = rule(
    implementation = py_env_rule_impl,
    attrs = {
        "deps": attr.label_list(
            mandatory = True,
            aspects = [py_imports_aspect],
            providers = [PyInfo],
        ),
        "package_name": attr.string(
            mandatory = True,
        ),
    },
    executable = True,
)

def py_env(name = ".env", **kwargs):
    py_env_rule(name = name, package_name = native.package_name(), **kwargs)
