## Description

The package provides a wrapper for pytest to be using conjunction with ROS.

To use the `ros_pytest_runner` use the following stanza in your `.test` launch file:

```xml
<launch>
  <param name="test_module" value="$(find your_package)/src"/>
  <test test-name="test_lib" pkg="ros_pytest" type="ros_pytest_runner" />
</launch>
```

Make sure to pass your test path as parameter `test_module` to the runner.

A more elaborate description for why I created this ROS package can be found on [my blog](https://machinekoder.com/testing-ros-powered-robots-pytest/).

## Run

To run the unit tests use following command inside your ROS package `catkin run_tests --this`.

Then view the test results with: `catkin_test_results`
