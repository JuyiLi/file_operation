cmake_minimum_required(VERSION 3.5)
project(file_operation)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-std=c++11)
add_compile_options(-fPIC)

find_package(Qt5 COMPONENTS Widgets LinguistTools REQUIRED)

SET(INCLUDE_DIR 

    ${CMAKE_CURRENT_SOURCE_DIR}/
	${CMAKE_CURRENT_SOURCE_DIR}/../../include
#	${CMAKE_CURRENT_SOURCE_DIR}/../../include/innfos_include
#	${CMAKE_CURRENT_SOURCE_DIR}/../../include/ARC_communication_new
#	${CMAKE_CURRENT_SOURCE_DIR}/../../include/jaka_zu_communication
	/usr/include/python2.7
	/usr/local/include
)
include_directories(${INCLUDE_DIR})

SET(LIB_DIR 

	${CMAKE_CURRENT_SOURCE_DIR}/../../lib
#	${CMAKE_CURRENT_SOURCE_DIR}/../../lib/ARC_communication_new
	/usr/local/lib
	
)
link_directories(${LIB_DIR})

SET(INCLUDE_FILES 

	${CMAKE_PROJECT_NAME}.h
)

add_library(${CMAKE_PROJECT_NAME} SHARED ${CMAKE_PROJECT_NAME}.cpp ${INCLUDE_FILES})

SET(test_list

    test
    )
foreach(var ${test_list})
    add_executable(${CMAKE_PROJECT_NAME}_${var} ${var}.cpp ${CMAKE_PROJECT_NAME}.cpp ${INCLUDE_FILES})

    target_link_libraries(${CMAKE_PROJECT_NAME}_${var} curve_fitting robotarm_kinematic)
endforeach()

add_custom_target(copyfile ALL
    COMMAND ${CMAKE_COMMAND} -E copy ${INCLUDE_FILES} ${CMAKE_CURRENT_SOURCE_DIR}/../../include
    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/build/lib${CMAKE_PROJECT_NAME}.so ${CMAKE_CURRENT_SOURCE_DIR}/../../lib
#  COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/*config.txt ${CMAKE_CURRENT_SOURCE_DIR}/../../conf
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "copy head files to include diretory; copy lib files to lib directory"
)

