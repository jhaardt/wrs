# coding=utf-8
import os
from distutils.version import LooseVersion
import pytest
import rospy


def get_output_file(argv):
    for arg in argv:
        if arg.startswith('--gtest_output'):
            return arg.split('=xml:')[1]

    raise RuntimeError('No output file has been passed')


def get_additional_args(argv):
    args = []
    for arg in argv[1:]:
        if arg.startswith('__') or arg.startswith('--gtest_output'):
            continue
        args.append(arg)
    return args


def create_cache_dir_args(version, output_file):
    # disable cache for Pytest < 3.2
    if LooseVersion("3.5.0") > LooseVersion(version):
        cache_dir_arg = '-p no:cacheprovider'
    else:
        root_dir = os.path.dirname(output_file)
        cache_dir_arg = '--rootdir={}'.format(root_dir)
    return cache_dir_arg.split(' ')


def run_pytest(argv):
    output_file = get_output_file(argv)
    additional_args = get_additional_args(argv)
    test_module = rospy.get_param('test_module')
    module_path = os.path.realpath(test_module)
    cache_dir_args = create_cache_dir_args(pytest.__version__, output_file)

    return pytest.main(
        [module_path, '--junitxml={}'.format(output_file)]
        + cache_dir_args
        + additional_args
    )
