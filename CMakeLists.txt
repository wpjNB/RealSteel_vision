cmake_minimum_required (VERSION 3.22)
project (ARMOR)

find_package(OpenCV REQUIRED)
# 添加openCV头文件所在目录
include_directories(
    /usr/local/include

    Armor/
)
include_directories(
    ${OpenCV_INCLUDE_DIRS}
)

aux_source_directory(./Armor/ SRCS2)
add_executable (main src/main.cpp ${SRCS2}
)
# 设置库文件的路径

target_link_libraries(main ${OpenCV_LIBS} )
