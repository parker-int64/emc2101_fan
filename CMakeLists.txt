cmake_minimum_required(VERSION 3.10)
project(emc2101 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(USE_WIRINGPI "Link wiringPi (for Raspberry Pi)" ON)
option(BUILD_SHARED_LIBS "Build shared libraries (if ON, build emc2101 as shared)" OFF)

# Options to use FetchContent to obtain dependencies from GitHub
include(FetchContent)
option(USE_FETCH_SPDLOG "Download spdlog via FetchContent if not found" ON)
option(USE_FETCH_CPPHTTPLIB "Download cpp-httplib via FetchContent" ON)


# Source files
set(EMC2101_SOURCES
	emc2101/src/emc2101.cpp
)
set(EMC2101_HEADERS
	emc2101/src/emc2101.h
)

set(EMC2101_SERVER_SOURCES
	emc2101/src/emc2101_server.cpp
)
set(EMC2101_SERVER_HEADERS
	emc2101/src/emc2101_server.h
)

# Library target for the device logic
add_library(emc2101 ${EMC2101_SOURCES} ${EMC2101_HEADERS})
target_include_directories(emc2101 PUBLIC ${PROJECT_SOURCE_DIR}/emc2101  ${PROJECT_SOURCE_DIR}/wiringOP/wiringPi)


# Prefer spdlog from find_package, but allow fetching it with FetchContent
if(USE_FETCH_SPDLOG)
    # FetchContent will provide spdlog::spdlog if it supplies a CMake config
    FetchContent_Declare(
        spdlog
        GIT_REPOSITORY https://github.com/gabime/spdlog.git
        GIT_TAG v1.11.0
    )
    FetchContent_MakeAvailable(spdlog)
    if(TARGET spdlog::spdlog)
        target_link_libraries(emc2101 PUBLIC spdlog::spdlog)
    endif()
else()
    find_package(spdlog CONFIG QUIET)
    if(spdlog_FOUND)
        target_link_libraries(emc2101 PUBLIC spdlog::spdlog)
    else()
        find_library(SPDLOG_LIB spdlog)
        if(SPDLOG_LIB)
            message(STATUS "Linking against system spdlog library: ${SPDLOG_LIB}")
            target_link_libraries(emc2101 PUBLIC ${SPDLOG_LIB})
        else()
            message(WARNING "spdlog not found. Please install spdlog or enable USE_FETCH_SPDLOG to download it.")
        endif()
    endif()
endif()

# Prefer cpp-httplib via FetchContent, otherwise try to find a system header
if(USE_FETCH_CPPHTTPLIB)
    FetchContent_Declare(
        cpp-httplib
        GIT_REPOSITORY https://github.com/yhirose/cpp-httplib.git
        GIT_TAG master
    )
    FetchContent_MakeAvailable(cpp-httplib)

    # If the fetched project provides a CMake target, link it; otherwise add include dirs
    if(TARGET cpp-httplib::cpp-httplib)
        target_link_libraries(emc2101 PUBLIC cpp-httplib::cpp-httplib)
    elseif(TARGET httplib OR TARGET cpphttplib OR TARGET cpp_httplib)
        # fallback for any possible target name
        target_link_libraries(emc2101 PUBLIC $<TARGET_NAME_IF_EXISTS:httplib>)
    else()
        # Add likely include locations from the fetched source
        if(DEFINED cpp-httplib_SOURCE_DIR)
            target_include_directories(emc2101 PUBLIC
                ${cpp-httplib_SOURCE_DIR}/single_include
                ${cpp-httplib_SOURCE_DIR}/include
                ${cpp-httplib_SOURCE_DIR}
            )
        endif()
    endif()
else()
    # Try to find a system-installed httplib.h
    find_path(CPPHTTPLIB_INCLUDE_DIR httplib.h)
    if(CPPHTTPLIB_INCLUDE_DIR)
        target_include_directories(emc2101 PUBLIC ${CPPHTTPLIB_INCLUDE_DIR})
    else()
        message(WARNING "cpp-httplib (httplib.h) not found. Please install it or enable USE_FETCH_CPPHTTPLIB to download it.")
    endif()
endif()



# Server executable
add_executable(emc2101_server ${EMC2101_SERVER_SOURCES})
target_link_libraries(emc2101_server PRIVATE emc2101)
target_include_directories(emc2101_server PRIVATE ${PROJECT_SOURCE_DIR}/emc2101 ${PROJECT_SOURCE_DIR})

# httplib is header-only in many setups; allow the project root to be used for locating it
# If you put httplib.h in the project root or add it to a system include path it will be found.

find_package(Threads REQUIRED)
target_link_libraries(emc2101_server PRIVATE Threads::Threads)

# Install rules
install(TARGETS emc2101 emc2101_server
		RUNTIME DESTINATION bin
		LIBRARY DESTINATION lib
		ARCHIVE DESTINATION lib)

message(STATUS "Configuration summary:")
message(STATUS "  Project: ${PROJECT_NAME}")
message(STATUS "  C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  Using wiringPi: ${USE_WIRINGPI}")


enable_testing()

find_package(Threads REQUIRED)
add_definitions(-D_WIN32_WINNT=0x0B00)
add_definitions(-DWINVER=0x0B00)

# Build a test executable that uses the test/server.cpp and the project's server implementation.
# server.cpp in test/ depends on emc2101/emc2101_server.cpp/.h so compile that source into the test.
add_executable(emc2101_server_test
    test/server.cpp
    emc2101/src/emc2101_server.cpp
)

# Ensure the test sees the project's headers
target_include_directories(emc2101_server_test PRIVATE
    ${PROJECT_SOURCE_DIR}/src/emc2101
    # ${PROJECT_SOURCE_DIR}
    # ${PROJECT_SOURCE_DIR}/wiringOP/wiringPi
)

# Link against the device library and threading support (and any transitive deps provided by emc2101)
target_link_libraries(emc2101_server_test PRIVATE
    emc2101
    Threads::Threads
)

# Register the test with CTest
add_test(NAME emc2101_server_test COMMAND emc2101_server_test)
set_tests_properties(emc2101_server_test PROPERTIES TIMEOUT 30)