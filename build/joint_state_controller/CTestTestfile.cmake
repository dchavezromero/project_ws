# CMake generated Testfile for 
# Source directory: /home/dennis/project_ws/src/ros_controllers/joint_state_controller
# Build directory: /home/dennis/project_ws/build/joint_state_controller
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_joint_state_controller_rostest_test_joint_state_controller.test "/home/dennis/project_ws/build/joint_state_controller/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/dennis/project_ws/build/joint_state_controller/test_results/joint_state_controller/rostest-test_joint_state_controller.xml" "--return-code" "/usr/bin/python2 /opt/ros/melodic/share/rostest/cmake/../../../bin/rostest --pkgdir=/home/dennis/project_ws/src/ros_controllers/joint_state_controller --package=joint_state_controller --results-filename test_joint_state_controller.xml --results-base-dir \"/home/dennis/project_ws/build/joint_state_controller/test_results\" /home/dennis/project_ws/src/ros_controllers/joint_state_controller/test/joint_state_controller.test ")
set_tests_properties(_ctest_joint_state_controller_rostest_test_joint_state_controller.test PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/melodic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/melodic/share/rostest/cmake/rostest-extras.cmake;52;catkin_run_tests_target;/opt/ros/melodic/share/rostest/cmake/rostest-extras.cmake;80;add_rostest;/opt/ros/melodic/share/rostest/cmake/rostest-extras.cmake;100;_add_rostest_google_test;/home/dennis/project_ws/src/ros_controllers/joint_state_controller/CMakeLists.txt;26;add_rostest_gtest;/home/dennis/project_ws/src/ros_controllers/joint_state_controller/CMakeLists.txt;0;")
subdirs("gtest")
