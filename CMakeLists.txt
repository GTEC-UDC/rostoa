cmake_minimum_required(VERSION 2.8.3)
project(gtec_rostoa)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -std=c++11")

add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  gtec_msgs
)

## System dependencies are found with CMake's conventions

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost REQUIRED COMPONENTS system)

find_package(Armadillo REQUIRED)
include_directories(${ARMADILLO_INCLUDE_DIRS})


## Generate messages in the 'msg' folder
#add_message_files(
#  FILES
# )

## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#  std_msgs
#)

catkin_package(
)

###########
## Build ##
###########

include_directories(src/rostoa/fixer)
include_directories(src/rostoa/fixer/nn)
include_directories(src/rostoa/publishers)
include_directories(src/rostoa/bias)
include_directories(src/rostoa/anchors)
include_directories(src/rostoa/diff)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(SOURCE_FILES_UWB_FIXER src/rostoa/publishers/node_uwb_fixer.cpp
    src/rostoa/fixer/UWBFixer.cpp
    src/rostoa/bias/BiasManager.cpp)

set(SOURCE_FILES_GENERIC_RANGING_FIXER src/rostoa/publishers/node_generic_ranging_fixer.cpp
    src/rostoa/fixer/GenericRangingFixer.cpp
    src/rostoa/bias/BiasManager.cpp)

set(SOURCE_FILES_DW_FIXER src/rostoa/publishers/node_dw_fixer.cpp
    src/rostoa/fixer/DWFixer.cpp
    src/rostoa/bias/BiasManager.cpp)

set(SOURCE_FILES_ANCHORS src/rostoa/publishers/node_anchors.cpp
  src/rostoa/anchors/AnchorPublisher.cpp)

set(SOURCE_FILES_POZYX_FIXER src/rostoa/publishers/node_pozyx_fixer.cpp
    src/rostoa/fixer/PozyxFixer.cpp)

set(SOURCE_FILES_RANGING_DIFFERENCE src/rostoa/publishers/node_ranging_diff.cpp
    src/rostoa/diff/RangingDifference.cpp)

set(SOURCE_FILES_NN_FIXER 
    src/rostoa/fixer/nn/nnClassifier.cpp
    src/rostoa/fixer/nn/nnClassifier_initialize.cpp
    src/rostoa/fixer/nn/nnClassifier_terminate.cpp
    src/rostoa/fixer/nn/nnMitigatorLOS.cpp
    src/rostoa/fixer/nn/nnMitigatorLOS_initialize.cpp
    src/rostoa/fixer/nn/nnMitigatorLOS_terminate.cpp
    src/rostoa/fixer/nn/repmat.cpp
    src/rostoa/publishers/node_nn_fixer.cpp
    src/rostoa/fixer/NNFixer.cpp)

## Declare a cpp executable

add_executable(uwbfixer ${SOURCE_FILES_UWB_FIXER})
target_link_libraries(uwbfixer
   ${catkin_LIBRARIES}
 )
add_dependencies(uwbfixer ${catkin_EXPORTED_TARGETS})
add_dependencies(uwbfixer gtec_msgs_generate_messages_cpp)


add_executable(genericrangingfixer ${SOURCE_FILES_GENERIC_RANGING_FIXER})
target_link_libraries(genericrangingfixer
   ${catkin_LIBRARIES}
 )
add_dependencies(genericrangingfixer ${catkin_EXPORTED_TARGETS})
add_dependencies(genericrangingfixer gtec_msgs_generate_messages_cpp)


add_executable(dwfixer ${SOURCE_FILES_DW_FIXER})
target_link_libraries(dwfixer
   ${catkin_LIBRARIES}
 )
add_dependencies(dwfixer ${catkin_EXPORTED_TARGETS})
add_dependencies(dwfixer gtec_msgs_generate_messages_cpp)


add_executable(pozyxfixer ${SOURCE_FILES_POZYX_FIXER})
target_link_libraries(pozyxfixer
   ${catkin_LIBRARIES}
 )
add_dependencies(pozyxfixer ${catkin_EXPORTED_TARGETS})
add_dependencies(pozyxfixer gtec_msgs_generate_messages_cpp)


add_executable(rangingdifference ${SOURCE_FILES_RANGING_DIFFERENCE})
target_link_libraries(rangingdifference
   ${catkin_LIBRARIES}
 )
add_dependencies(rangingdifference ${catkin_EXPORTED_TARGETS})
add_dependencies(rangingdifference gtec_msgs_generate_messages_cpp)

add_executable(anchors ${SOURCE_FILES_ANCHORS})
target_link_libraries(anchors
   ${catkin_LIBRARIES}
 )
add_dependencies(anchors ${catkin_EXPORTED_TARGETS})

add_executable(nnfixer ${SOURCE_FILES_NN_FIXER})
target_link_libraries(nnfixer
   ${catkin_LIBRARIES}
 )
add_dependencies(nnfixer ${catkin_EXPORTED_TARGETS})
add_dependencies(nnfixer gtec_msgs_generate_messages_cpp)




