add_test( SubgoalGenTest.SubgoalGen_TEST /home/changju/cpp_ws/subgoal_generator/build/subgoal_generator_test [==[--gtest_filter=SubgoalGenTest.SubgoalGen_TEST]==] --gtest_also_run_disabled_tests)
set_tests_properties( SubgoalGenTest.SubgoalGen_TEST PROPERTIES WORKING_DIRECTORY /home/changju/cpp_ws/subgoal_generator/build)
set( subgoal_generator_test_TESTS SubgoalGenTest.SubgoalGen_TEST)
