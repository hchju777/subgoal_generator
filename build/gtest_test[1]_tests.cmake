add_test( gtestTest.TestCopy /home/changju/cpp_ws/subgoal_generator/build/gtest_test [==[--gtest_filter=gtestTest.TestCopy]==] --gtest_also_run_disabled_tests)
set_tests_properties( gtestTest.TestCopy PROPERTIES WORKING_DIRECTORY /home/changju/cpp_ws/subgoal_generator/build)
set( gtest_test_TESTS gtestTest.TestCopy)
