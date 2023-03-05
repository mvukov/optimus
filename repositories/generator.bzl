""" Defines a custom rule that executes a generator in the exec config.
"""

def _generator_impl(ctx):
    output_file = ctx.actions.declare_file(ctx.attr.name + ctx.attr.extension)
    args = ctx.actions.args()
    args.add_all(ctx.attr.args)
    args.add("--output", output_file)
    ctx.actions.run(
        inputs = [],
        outputs = [output_file],
        arguments = [args],
        executable = ctx.executable.generator,
        mnemonic = "OptimusGenerator",
        progress_message = "Running generator for %{label}",
    )
    return [DefaultInfo(files = depset([output_file]))]

generator = rule(
    implementation = _generator_impl,
    attrs = {
        "extension": attr.string(
            default = ".h",
            doc = "The extension of the output file.",
        ),
        "args": attr.string_list(
            doc = "The generator arguments.",
        ),
        "generator": attr.label(
            executable = True,
            cfg = "exec",
            doc = "The generator executable.",
        ),
    },
)
