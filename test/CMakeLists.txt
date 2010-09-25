INCLUDE_DIRECTORIES (.)

ADD_EXECUTABLE (graph_test graph_test.cc)
ADD_TEST (graph_test graph_test)

ADD_EXECUTABLE (bgl_adaptor_test bgl_adaptor_test.cc)
ADD_TEST (bgl_adaptor_test bgl_adaptor_test)

ADD_EXECUTABLE (algorithm_test algorithm_test.cc)
SET_TARGET_PROPERTIES(algorithm_test PROPERTIES
    COMPILE_FLAGS "-std=c++0x")
ADD_TEST (algorithm_test algorithm_test)

