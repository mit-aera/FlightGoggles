add_executable(GeneralClient GeneralClient.cpp)
target_link_libraries(GeneralClient  ${OpenCV_LIBS} FlightGogglesClientLib pthread)
