# Add FlightGogglesClient as library
add_library(FlightGogglesClientLib SHARED FlightGogglesClient.cpp FlightGogglesClient.hpp)

# Link in needed libraries
target_link_libraries(FlightGogglesClientLib zmq zmqpp ${OpenCV_LIBS} pthread)

# Expose as library
target_include_directories(FlightGogglesClientLib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
