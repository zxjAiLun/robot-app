# CMake generated Testfile for 
# Source directory: /home/ubuntu/DishBot/src/disbot_controller
# Build directory: /home/ubuntu/DishBot/build/disbot_controller
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(lint_cmake "/usr/bin/python3" "-u" "/opt/ros/jazzy/share/ament_cmake_test/cmake/run_test.py" "/home/ubuntu/DishBot/build/disbot_controller/test_results/disbot_controller/lint_cmake.xunit.xml" "--package-name" "disbot_controller" "--output-file" "/home/ubuntu/DishBot/build/disbot_controller/ament_lint_cmake/lint_cmake.txt" "--command" "/opt/ros/jazzy/bin/ament_lint_cmake" "--xunit-file" "/home/ubuntu/DishBot/build/disbot_controller/test_results/disbot_controller/lint_cmake.xunit.xml")
set_tests_properties(lint_cmake PROPERTIES  LABELS "lint_cmake;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/ubuntu/DishBot/src/disbot_controller" _BACKTRACE_TRIPLES "/opt/ros/jazzy/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/jazzy/share/ament_cmake_lint_cmake/cmake/ament_lint_cmake.cmake;47;ament_add_test;/opt/ros/jazzy/share/ament_cmake_lint_cmake/cmake/ament_cmake_lint_cmake_lint_hook.cmake;21;ament_lint_cmake;/opt/ros/jazzy/share/ament_cmake_lint_cmake/cmake/ament_cmake_lint_cmake_lint_hook.cmake;0;;/opt/ros/jazzy/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/jazzy/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/jazzy/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/jazzy/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/jazzy/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/ubuntu/DishBot/src/disbot_controller/CMakeLists.txt;26;ament_package;/home/ubuntu/DishBot/src/disbot_controller/CMakeLists.txt;0;")
add_test(xmllint "/usr/bin/python3" "-u" "/opt/ros/jazzy/share/ament_cmake_test/cmake/run_test.py" "/home/ubuntu/DishBot/build/disbot_controller/test_results/disbot_controller/xmllint.xunit.xml" "--package-name" "disbot_controller" "--output-file" "/home/ubuntu/DishBot/build/disbot_controller/ament_xmllint/xmllint.txt" "--command" "/opt/ros/jazzy/bin/ament_xmllint" "--xunit-file" "/home/ubuntu/DishBot/build/disbot_controller/test_results/disbot_controller/xmllint.xunit.xml")
set_tests_properties(xmllint PROPERTIES  LABELS "xmllint;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/ubuntu/DishBot/src/disbot_controller" _BACKTRACE_TRIPLES "/opt/ros/jazzy/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/jazzy/share/ament_cmake_xmllint/cmake/ament_xmllint.cmake;50;ament_add_test;/opt/ros/jazzy/share/ament_cmake_xmllint/cmake/ament_cmake_xmllint_lint_hook.cmake;18;ament_xmllint;/opt/ros/jazzy/share/ament_cmake_xmllint/cmake/ament_cmake_xmllint_lint_hook.cmake;0;;/opt/ros/jazzy/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/jazzy/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/jazzy/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/jazzy/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/jazzy/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/ubuntu/DishBot/src/disbot_controller/CMakeLists.txt;26;ament_package;/home/ubuntu/DishBot/src/disbot_controller/CMakeLists.txt;0;")
