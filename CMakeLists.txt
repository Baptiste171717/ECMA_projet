project(Projet_ECMA_Baptiste_Vert LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set the target architecture to 64-bit
set(CMAKE_GENERATOR_PLATFORM x64)

# Specify the runtime library for all configurations
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MD")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MDd")

# Add the CPLEX include directories
include_directories("C:/Program Files/IBM/ILOG/CPLEX_Studio2211/cplex/include")
include_directories("C:/Program Files/IBM/ILOG/CPLEX_Studio2211/concert/include")

# Add the CPLEX library directories for debug build
link_directories("C:/Program Files/IBM/ILOG/CPLEX_Studio2211/cplex/lib/x64_windows_msvc14/stat_mdd")
link_directories("C:/Program Files/IBM/ILOG/CPLEX_Studio2211/concert/lib/x64_windows_msvc14/stat_mdd")

# Add the executable
add_executable(Projet_ECMA_Baptiste_Vert main.cpp
    ECMA.h
    ECMA.cpp
    write_files.cpp
    write_files.h
    general_classes.h
    ECMA_heuristic.h
    ECMA_heuristic.cpp)

# Link the CPLEX libraries
target_link_libraries(Projet_ECMA_Baptiste_Vert ilocplex concert cplex2211)
