find_package(GTest REQUIRED)

add_executable(${PROJECT_NAME}_test_filter test_filter.cpp)
target_link_libraries(${PROJECT_NAME}_test_filter GTest::GTest GTest::Main)
add_test(test_filter ${PROJECT_NAME}_test_filter)

add_executable(${PROJECT_NAME}_test_ukf test_ukf.cpp)
target_link_libraries(${PROJECT_NAME}_test_ukf GTest::GTest GTest::Main)
add_test(test_ukf ${PROJECT_NAME}_test_ukf)
