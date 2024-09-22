set(SOURCES
    map_generation_test.cpp
)

catkin_add_gtest(map_generation_test ${SOURCES})

target_link_libraries(map_generation_test
    ${catkin_LIBRARIES}
    ${GTEST_LIBRARIES}
    map_generation
)
