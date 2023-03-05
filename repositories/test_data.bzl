""" Handles import of external repositories with test data.
"""

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_file")

def test_data():
    http_file(
        name = "test_data_csail",
        urls = ["http://www2.informatik.uni-freiburg.de/%7Estachnis/datasets/datasets/csail/scsail.corrected.png"],
        sha256 = "6b85d7f44a1fa7e0ce0edc72f54fc64db6caa4f76118fcc1c7004ac666dda37f",
        downloaded_file_path = "scsail_corrected.png",
    )
