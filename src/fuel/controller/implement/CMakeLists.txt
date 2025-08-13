add_library(controller_algorithms SHARED 
  src/lqr.cpp
  src/mpc.cpp
  src/mpc_casadi.cpp
)

# TODO IPOPT library (Consider moving to /user/lib)
add_library(ipopt SHARED IMPORTED)
set_target_properties(ipopt PROPERTIES
  IMPORTED_LOCATION "/usr/local/lib/libipopt.so"
  INTERFACE_INCLUDE_DIRECTORIES "/usr/local/include/coin-or"
)

find_path(CASADI_INCLUDE_DIR casadi/casadi.hpp PATHS /usr/local/include)
find_library(CASADI_LIBRARY casadi PATHS /usr/local/lib)

if(NOT CASADI_INCLUDE_DIR OR NOT CASADI_LIBRARY)
    message(FATAL_ERROR "CasADi not found in /usr/local")
endif()

target_include_directories(controller_algorithms
    SYSTEM PUBLIC
        ${CASADI_INCLUDE_DIR}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(controller_algorithms
    ${CASADI_LIBRARY}
    ipopt
)

ament_target_dependencies(controller_algorithms
  Eigen3
  utilities
  dsm
)