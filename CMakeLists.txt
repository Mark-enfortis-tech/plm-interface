cmake_minimum_required(VERSION 3.10)
project(PlmInterfaceCpp CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add include directory
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Define common source files (shared by both executables)
set(COMMON_SOURCES
    src/Message.cpp
    src/PlmTxHandler.cpp
    src/PlmRxHandler.cpp
    src/Serial.cpp
)

# Create a static library for common code (optional but recommended)
add_library(plm_common STATIC ${COMMON_SOURCES})

# Define source files for executables (no PlmInterface.cpp needed)
set(TEST_SENDER_SOURCES
    test/TestSender.cpp
)

set(TEST_RECEIVER_SOURCES
    test/TestReceiver.cpp
)

# Create executables
add_executable(test_sender ${TEST_SENDER_SOURCES})
add_executable(test_receiver ${TEST_RECEIVER_SOURCES})

# Link executables with the common library and required system libraries
target_link_libraries(test_sender plm_common pthread)
target_link_libraries(test_receiver plm_common pthread)

# Set output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Add debug configuration
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -Wextra -g")

# Add release configuration with optimizations
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -DNDEBUG")

# Install targets
install(TARGETS test_sender test_receiver plm_common
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

# Install scripts
install(FILES 
    scripts/setup_virtual_ports.sh
    PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
    DESTINATION bin
)

# Add custom target for running tests
add_custom_target(run_test
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/scripts/setup_virtual_ports.sh
    DEPENDS test_sender test_receiver
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Setting up virtual ports and running PLM test"
)

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
