""" Configures repo dependencies.
"""

load("@bazel_skylib//:workspace.bzl", "bazel_skylib_workspace")
load("@com_github_mvukov_qpoases_embedded//:deps.bzl", "qpoases_embedded_deps")
load("@com_github_mvukov_qpoases_embedded//:repositories.bzl", "qpoases_embedded_repositories")
load("@com_github_nelhage_rules_boost//:boost/boost.bzl", "boost_deps")

def optimus_deps():
    bazel_skylib_workspace()

    qpoases_embedded_repositories()
    qpoases_embedded_deps()

    boost_deps()
