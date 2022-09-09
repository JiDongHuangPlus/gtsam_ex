project(test)
cmake_minimum_required(VERSION 3.1)

# 寻找第三方库，使用大小写都可以，这里列举了两种方式
find_package(Boost COMPONENTS thread filesystem date_time system REQUIRED)
find_package(GTSAM REQUIRED)

# 包含第三方库头文件路径，可以使用绝对路径
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${GTSAM_INCLUDE_DIR})
INCLUDE_DIRECTORIES("/usr/include/eigen3")

add_executable(robot_motion robot_motion.cpp)
add_executable(localization robot_localization.cpp)
add_executable(landmark landmark.cpp)
add_executable(sfm SFM.cpp)

# 链接库 
target_link_libraries(robot_motion ${Boost_LIBRARIES} -lgtsam -ltbb)
install(TARGETS robot_motion RUNTIME DESTINATION bin)
target_link_libraries(localization ${Boost_LIBRARIES} -lgtsam -ltbb)
install(TARGETS localization RUNTIME DESTINATION bin)
target_link_libraries(landmark ${Boost_LIBRARIES} -lgtsam -ltbb)
install(TARGETS landmark RUNTIME DESTINATION bin)
target_link_libraries(sfm ${Boost_LIBRARIES} -lgtsam -ltbb)
install(TARGETS sfm RUNTIME DESTINATION bin)