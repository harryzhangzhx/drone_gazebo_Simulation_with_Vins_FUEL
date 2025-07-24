add_executable(governor_example governor_example.cpp)

target_include_directories(governor_example
  PRIVATE
    ../interface/include
)

target_link_libraries(governor_example governor)

install(
    TARGETS governor_example
    DESTINATION bin
)