cmake_minimum_required(VERSION 3.10)
project(MPCC)

set(CMAKE_CXX_FLAGS "-O2")

option(USE_CVPLOT "Use cv-plot library" ON)
MESSAGE( STATUS "Using CvPlot: " ${USE_CVPLOT} )

find_package(RBDL REQUIRED)
find_package(OsqpEigen REQUIRED)
include_directories(.)

add_library(MPCC_LIB SHARED
        config.h
        types.cpp
        types.h
        Model/robot_model.cpp
        Model/robot_model.h
        Constraints/SelfCollision/SelfCollisionModel.cpp
        Constraints/SelfCollision/SelfCollisionModel.h
        Interfaces/osqp_interface.cpp
        Interfaces/solver_interface.cpp
        Model/model.cpp
        Model/integrator.cpp
        Spline/cubic_spline.cpp
        Spline/cubic_spline_rot.cpp
        Spline/arc_length_spline.cpp
        Params/params.cpp
        Constraints/constraints.cpp
        Cost/cost.cpp
        Constraints/bounds.cpp
        MPC/mpc.cpp
        Params/track.cpp
        Params/track.h
        )
target_link_libraries(MPCC_LIB PUBLIC
        ${RBDL_LIBRARY} 
        OsqpEigen::OsqpEigen 
)

add_executable(MPCC_EXE main.cpp)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
target_link_libraries(MPCC_EXE MPCC_LIB)

if(USE_CVPLOT)
        find_package (OpenCV REQUIRED)
        include_directories(${OpenCV_INCLUDE_DIRS})
        include_directories(External/cv-plot/lib/include)
        target_link_libraries(MPCC_EXE ${OpenCV_LIBS} ${CMAKE_SOURCE_DIR}/MPCC/External/cv-plot/lib/lib/libCvPlot.so)
        target_compile_definitions(MPCC_EXE PRIVATE USE_CVPLOT)
endif()

add_executable(MPCC_TEST
        Tests/main_test.cpp
        Tests/model_integrator_test.h
        Tests/robot_model_test.h
        Tests/self_collision_test.h
        Tests/spline_test.h
        Tests/constraints_test.h
        Tests/cost_test.h
)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
find_package(Python COMPONENTS Development)
target_include_directories(MPCC_TEST PRIVATE ${Python_INCLUDE_DIRS})
target_link_libraries(MPCC_TEST PRIVATE MPCC_LIB gtest gtest_main pthread ${Python_LIBRARIES})