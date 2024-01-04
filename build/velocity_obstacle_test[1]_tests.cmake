add_test( VOTest.VOConeGen_TEST /home/changju/cpp_ws/subgoal_generator/build/velocity_obstacle_test [==[--gtest_filter=VOTest.VOConeGen_TEST]==] --gtest_also_run_disabled_tests)
set_tests_properties( VOTest.VOConeGen_TEST PROPERTIES WORKING_DIRECTORY /home/changju/cpp_ws/subgoal_generator/build)
set( velocity_obstacle_test_TESTS VOTest.VOConeGen_TEST)
