find_package(GTest REQUIRED)

add_executable(${PROJECT_NAME}_test_filter test_filter.cpp)
target_link_libraries(${PROJECT_NAME}_test_filter GTest::GTest GTest::Main)
add_test(test_filter ${PROJECT_NAME}_test_filter)

add_executable(${PROJECT_NAME}_test_unscented_transform test_unscented_transform.cpp)
target_link_libraries(${PROJECT_NAME}_test_unscented_transform GTest::GTest GTest::Main)
add_test(test_unscented_transform ${PROJECT_NAME}_test_unscented_transform)

add_executable(${PROJECT_NAME}_test_kalman_filter test_kalman_filter.cpp)
target_link_libraries(${PROJECT_NAME}_test_kalman_filter GTest::GTest GTest::Main)
add_test(test_kalman_filter ${PROJECT_NAME}_test_kalman_filter)

add_executable(${PROJECT_NAME}_test_gaussian_types test_gaussian_types.cpp)
target_link_libraries(${PROJECT_NAME}_test_gaussian_types GTest::GTest GTest::Main)
add_test(test_gaussian_types ${PROJECT_NAME}_test_gaussian_types)

add_executable(${PROJECT_NAME}_test_kalman_equations test_kalman_equations.cpp)
target_link_libraries(${PROJECT_NAME}_test_kalman_equations GTest::GTest GTest::Main)
add_test(test_kalman_equations ${PROJECT_NAME}_test_kalman_equations)

add_executable(${PROJECT_NAME}_test_particle_filter test_particle_filter.cpp)
target_link_libraries(${PROJECT_NAME}_test_particle_filter GTest::GTest GTest::Main)
add_test(test_particle_filter ${PROJECT_NAME}_test_particle_filter)

add_executable(${PROJECT_NAME}_test_filter_base test_filter_base.cpp)
target_link_libraries(${PROJECT_NAME}_test_filter_base GTest::GTest GTest::Main)
add_test(test_filter_base ${PROJECT_NAME}_test_filter_base)
