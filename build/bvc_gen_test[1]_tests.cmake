add_test( BVCGenTest.VD_GEN_TEST /home/changju/cpp_ws/subgoal_generator/build/bvc_gen_test [==[--gtest_filter=BVCGenTest.VD_GEN_TEST]==] --gtest_also_run_disabled_tests)
set_tests_properties( BVCGenTest.VD_GEN_TEST PROPERTIES WORKING_DIRECTORY /home/changju/cpp_ws/subgoal_generator/build)
set( bvc_gen_test_TESTS BVCGenTest.VD_GEN_TEST)
