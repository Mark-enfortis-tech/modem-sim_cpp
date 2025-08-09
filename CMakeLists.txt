cmake_minimum_required(VERSION 3.10)
project(PlmSimulator CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add include directory
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Define source files
set(SIMULATOR_SOURCES
    src/main.cpp
    src/Message.cpp
    src/Serial.cpp
    src/Simulator.cpp
)

set(TEST_SENDER_SOURCES
    test/TestSender.cpp
    src/Message.cpp
)

set(TEST_RECEIVER_SOURCES
    test/TestReceiver.cpp
    src/Message.cpp
)

# Create executables
add_executable(plm_simulator ${SIMULATOR_SOURCES})
add_executable(test_sender ${TEST_SENDER_SOURCES})
add_executable(test_receiver ${TEST_RECEIVER_SOURCES})

# Link with required libraries
target_link_libraries(plm_simulator pthread)
target_link_libraries(test_sender pthread)
target_link_libraries(test_receiver pthread)

# Install targets
install(TARGETS plm_simulator test_sender test_receiver
    RUNTIME DESTINATION bin
)

# Install scripts
install(FILES 
    scripts/setup_virtual_ports.sh
    scripts/run_test.sh
    PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
    DESTINATION bin
)

# Add custom target for running tests
add_custom_target(run_test
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/scripts/run_test.sh
    DEPENDS plm_simulator test_sender test_receiver
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Running PLM simulator test"
)

# Add debug configuration
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -Wextra")

# Add release configuration with optimizations
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

# Output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Print configuration summary
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ compiler: ${CMAKE_CXX_COMPILER}")
message(STATUS "C++ flags: ${CMAKE_CXX_FLAGS}")
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    message(STATUS "Debug flags: ${CMAKE_CXX_FLAGS_DEBUG}")
elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
    message(STATUS "Release flags: ${CMAKE_CXX_FLAGS_RELEASE}")
endif()

# Add option for building documentation (if Doxygen is available)
option(BUILD_DOCUMENTATION "Build documentation (requires Doxygen)" OFF)
if(BUILD_DOCUMENTATION)
    find_package(Doxygen)
    if(DOXYGEN_FOUND)
        # Configure Doxygen file
        configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
        
        # Add documentation target
        add_custom_target(doc
            ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            COMMENT "Generating API documentation with Doxygen" VERBATIM
        )
    else()
        message(STATUS "Doxygen not found, documentation will not be built")
    endif()
endif()

# Add packaging support
include(InstallRequiredSystemLibraries)
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
set(CPACK_PACKAGE_VERSION_MAJOR "1")
set(CPACK_PACKAGE_VERSION_MINOR "0")
set(CPACK_PACKAGE_VERSION_PATCH "0")
include(CPack)
