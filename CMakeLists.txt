cmake_minimum_required(VERSION 3.26)
project(Practice LANGUAGES C CXX)

option(BUILD_SHARED_LIBS "Enable build shared lib" OFF)

set(CMAKE_CXX_STANDARD 11)

set(OpenCV_DIR "D:/library/opencv/build/x64/vc15/lib")
find_package(OpenCV REQUIRED)

include_directories(
  ${CMAKE_CURRENT_LIST_DIR}/include
  ${CMAKE_CURRENT_LIST_DIR}/source
  ${OpenCV_INCLUDE_DIRS}
  D:/library/eigen
)

file(GLOB_RECURSE SRC_FILES
  ${CMAKE_CURRENT_LIST_DIR}/include/*.h
  ${CMAKE_CURRENT_LIST_DIR}/source/*.cc
  ${CMAKE_CURRENT_LIST_DIR}/source/*.h
)

if (BUILD_SHARED_LIBS)
  add_library(${PROJECT_NAME} SHARED ${SRC_FILES})
else (BUILD_SHARED_LIBS)
  add_library(${PROJECT_NAME} STATIC ${SRC_FILES})
endif (BUILD_SHARED_LIBS)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

if(CMAKE_COMPILER_IS_GNUCXX)
  # 检查GNU编译是否支持 -Wa,-mbig-obj
  include(CheckCXXCompilerFlag)
  check_cxx_compiler_flag("-Wa,-mbig-obj" GNU_BIG_OBJ_FLAG_ENABLE)
  message(STATUS GNU_BIG_OBJ_FLAG_ENABLE=${GNU_BIG_OBJ_FLAG_ENABLE})
  target_link_libraries(${PROJECT_NAME} dl)
endif()

target_compile_options(${PROJECT_NAME}
  PRIVATE
  $<$<CXX_COMPILER_ID:MSVC>:/bigobj>
  $<$<AND:$<CXX_COMPILER_ID:GNU>,$<BOOL:${GNU_BIG_OBJ_FLAG_ENABLE}>>:-Wa,-mbig-obj>)

set(TEST_LIST
  TestHelloWorld
  TestTracker
)

foreach (TEST ${TEST_LIST})
  add_executable(${TEST} test/${TEST}.cc)
  target_link_libraries(${TEST} ${PROJECT_NAME})
endforeach (TEST ${TEST_LIST})
