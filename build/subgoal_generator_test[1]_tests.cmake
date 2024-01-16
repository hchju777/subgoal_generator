add_test( SubgoalGenTest.generate_subgoals_TEST /home/changju/cpp_ws/subgoal_generator/build/subgoal_generator_test [==[--gtest_filter=SubgoalGenTest.generate_subgoals_TEST]==] --gtest_also_run_disabled_tests)
set_tests_properties( SubgoalGenTest.generate_subgoals_TEST PROPERTIES WORKING_DIRECTORY /home/changju/cpp_ws/subgoal_generator/build)
set( subgoal_generator_test_TESTS SubgoalGenTest.generate_subgoals_TEST)
