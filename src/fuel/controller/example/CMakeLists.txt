add_executable(controller_example controller_example.cpp)

target_include_directories(controller_example
  PRIVATE
    ../interface/include
)

target_link_libraries(controller_example controller)

install(TARGETS 
    controller_example
    DESTINATION bin
)