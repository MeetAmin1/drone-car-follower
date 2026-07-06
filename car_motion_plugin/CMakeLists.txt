cmake_minimum_required(VERSION 3.16)
project(car_motion_plugin)

find_package(ament_cmake REQUIRED)
find_package(gz-sim8 REQUIRED)

add_library(${PROJECT_NAME} SHARED src/figure_eight_motion.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(${PROJECT_NAME} PRIVATE gz-sim8::gz-sim8)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_package()
