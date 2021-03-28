cmake_minimum_required(VERSION 2.8.3)
project(C-MINT)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -std=c++11")

#set(EIGEN_INCLUDE_DIR "/usr/include/eigen3")

find_package(catkin REQUIRED
  COMPONENTS
  cmake_modules)
find_package(ompl REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread filesystem graph program_options system)
find_package(OpenCV REQUIRED)
find_package(DART REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${DART_INCLUDE_DIRS}
  ${ompl_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

# Generic 2D example scipt. API - 2
add_executable(exampleTwoAgentsC_MINT examples/exampleTwoAgentsC_MINT.cpp)
target_link_libraries(exampleTwoAgentsC_MINT 
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
  ${DART_LIBRARIES}
  ${OMPL_LIBRARIES}
  )
install(TARGETS exampleTwoAgentsC_MINT RUNTIME DESTINATION bin)

# # Generic 2D example scipt. API - 2
# add_executable(exampleTwoAgentsC_MINT_2 examples/exampleTwoAgentsC_MINT_2.cpp)
# target_link_libraries(exampleTwoAgentsC_MINT_2
#   ${OpenCV_LIBS}
#   ${Boost_LIBRARIES}
#   ${DART_LIBRARIES}
#   ${OMPL_LIBRARIES}
#   )
# install(TARGETS exampleTwoAgentsC_MINT_2 RUNTIME DESTINATION bin)


# # Generic 2D example scipt. API - 2
# add_executable(exampleTwoAgentsDENSIFY examples/exampleTwoAgentsDENSIFY.cpp)
# target_link_libraries(exampleTwoAgentsDENSIFY
#   ${OpenCV_LIBS}
#   ${Boost_LIBRARIES}
#   ${DART_LIBRARIES}
#   ${OMPL_LIBRARIES}
#   )
# install(TARGETS exampleTwoAgentsDENSIFY RUNTIME DESTINATION bin)

# add_executable(exampleTwoAgentsTRIVIAL examples/exampleTwoAgentsTRIVIAL.cpp)
# target_link_libraries(exampleTwoAgentsTRIVIAL
#   ${OpenCV_LIBS}
#   ${Boost_LIBRARIES}
#   ${DART_LIBRARIES}
#   ${OMPL_LIBRARIES}
#   )
# install(TARGETS exampleTwoAgentsTRIVIAL RUNTIME DESTINATION bin)

# This custom target doesn't affect building the project but is only for
# displaying the header files in IDEs.
#FILE(GLOB_RECURSE LibFiles "include/*.hpp")
#add_custom_target(headers SOURCES ${LibFiles})

