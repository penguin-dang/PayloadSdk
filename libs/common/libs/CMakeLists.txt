##################### STANDARD HEADER #########################

cmake_minimum_required(VERSION 3.5)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

##################### PROJECT DEF #############################
project(common)

####################### INCLUDE   #############################

#find_package(Threads REQUIRED)

include_directories (${PROJECT_SOURCE_DIR}/inc)

# add_compile_options(-Wall -Wextra)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread") # thread

#######################  COMPILE  #############################
file(GLOB SOURCES "src/*.cpp")

# # Library build
# add_library(${PROJECT_NAME} STATIC ${SOURCES}) # Static by default

# target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/inc)
# # if(OCLEA_PLATFORM)
	
# # 	target_link_libraries(${PROJECT_NAME} std::filesystem)
# # elseif(GENIO_PLATFORM)
# # 	target_link_libraries(${PROJECT_NAME} std::filesystem)
# # else()
# # 	target_link_libraries(${PROJECT_NAME} stdc++fs)
# # endif(OCLEA_PLATFORM)

# target_link_libraries(${PROJECT_NAME} stdc++fs)


# Library build
add_library(${PROJECT_NAME} STATIC ${SOURCES}) # Static by default
target_include_directories(${PROJECT_NAME} PUBLIC
                           ${PROJECT_SOURCE_DIR}/inc
                           )
