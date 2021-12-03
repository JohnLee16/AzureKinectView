cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(AzureKinectView)

add_executable(AzureKinectView main.cpp)
# link_directories(/usr/local/lib)   'find_package' has already done this
#target_link_libraries(AzureKinectView "${TORCH_LIBRARIES}")
#target_link_libraries(AzureKinectView "${OpenCV_LIBS}")
set_property(TARGET AzureKinectView PROPERTY CXX_STANDARD 11)