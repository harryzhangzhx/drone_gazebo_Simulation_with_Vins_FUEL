add_executable(dsm_example dsm_example.cpp)

target_include_directories(dsm_example
  PRIVATE
    ../interface/include
)

target_link_libraries(dsm_example dsm)

install(
    TARGETS dsm_example
    DESTINATION bin
)