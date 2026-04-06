# CMake generated Testfile for 
# Source directory: /home/para/HAMR/HAMR_Controller/test/ekf_tests
# Build directory: /home/para/HAMR/HAMR_Controller/test/ekf_tests/build
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_ekf "/home/para/HAMR/HAMR_Controller/test/ekf_tests/build/test_ekf")
set_tests_properties(test_ekf PROPERTIES  _BACKTRACE_TRIPLES "/home/para/HAMR/HAMR_Controller/test/ekf_tests/CMakeLists.txt;34;add_test;/home/para/HAMR/HAMR_Controller/test/ekf_tests/CMakeLists.txt;0;")
add_test(test_odometry "/home/para/HAMR/HAMR_Controller/test/ekf_tests/build/test_odometry")
set_tests_properties(test_odometry PROPERTIES  _BACKTRACE_TRIPLES "/home/para/HAMR/HAMR_Controller/test/ekf_tests/CMakeLists.txt;44;add_test;/home/para/HAMR/HAMR_Controller/test/ekf_tests/CMakeLists.txt;0;")
subdirs("_deps/googletest-build")
