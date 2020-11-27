# -*- coding: utf-8 -*-
import pytest

import ros_pytest


def test_output_file_is_correctly_extracted_from_argv():
    output = ros_pytest.get_output_file(['runner.py', '--gtest_output=xml:~/junit_output.xml'])

    assert output == '~/junit_output.xml'


def test_additional_args_are_correctly_extracted_from_argv():
    output = ros_pytest.get_additional_args([
        'runner.py', '--gtest_output=xml:~/junit_output.xml',
        '--ignore=some_tests', '__name:=test_lib',
    ])

    assert output == ['--ignore=some_tests']


def test_not_passing_output_file_throws_runtime_error():
    with pytest.raises(RuntimeError):
        ros_pytest.get_output_file(['main.py', '--file', 'random.txt'])


def test_caching_is_disabled_for_older_pytest_versions():
    args = ros_pytest.create_cache_dir_args("3.0.1", "/tmp/output_file.xml")

    assert args == ['-p', 'no:cacheprovider']


def test_rootdir_is_set_for_newer_pytest_versions():
    args = ros_pytest.create_cache_dir_args("3.5.0", "/tmp/output_file.xml")

    assert args == ['--rootdir=/tmp']
