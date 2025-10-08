cmake_minimum_required(VERSION 3.20)
project(cwAsio4RtAudio VERSION 19.8)

include(FetchContent)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_SHARED_MODULE_PREFIX "")

set(CWASIO_SHARED OFF)
set(C4R_USE_CWASIO_REPO "https://github.com/s13n/cwASIO.git" CACHE STRING "cwASIO repository to use")
if(NOT C4R_USE_CWASIO_TAG)
    if($ENV{C4R_USE_CWASIO_TAG})
        set(C4R_USE_CWASIO_TAG $ENV{C4R_USE_CWASIO_TAG})
    else()
        set(C4R_USE_CWASIO_TAG "0bc9c06b50d588639af78c66ea3ef5245cc971b3")   # set default tag, when neither cache nor environment variable found.
    endif()
endif()

FetchContent_Declare(cwASIO_build
    GIT_TAG         ${C4R_USE_CWASIO_TAG}
    GIT_REPOSITORY  ${C4R_USE_CWASIO_REPO}
)
message(STATUS "Fetching cwASIO sources ${C4R_USE_CWASIO_TAG} from git at ${C4R_USE_CWASIO_REPO}")
FetchContent_GetProperties(cwASIO_build)
if(NOT cwasio_build_POPULATED)
    FetchContent_Populate(cwASIO_build)
    message(STATUS "Building cwASIO in ${cwasio_build_BINARY_DIR} from sources in ${cwasio_build_SOURCE_DIR}")
    add_subdirectory(${cwasio_build_SOURCE_DIR} ${cwasio_build_BINARY_DIR} EXCLUDE_FROM_ALL)
endif()

set(RTAUDIO_BUILD_STATIC_LIBS ON)
set(C4R_USE_RTAUDIO_REPO "https://github.com/thestk/rtaudio.git" CACHE STRING "RtAudio repository to use")
if(NOT C4R_USE_RTAUDIO_TAG)
    if($ENV{C4R_USE_RTAUDIO_TAG})
        set(C4R_USE_RTAUDIO_TAG $ENV{C4R_USE_RTAUDIO_TAG})
    else()
        set(C4R_USE_RTAUDIO_TAG "acec45bb17425b0bfbc9972ead29d2b0c22c0c93")   # set default tag, when neither cache nor environment variable found.
    endif()
endif()

FetchContent_Declare(RtAudio_build
    GIT_TAG         ${C4R_USE_RTAUDIO_TAG}
    GIT_REPOSITORY  ${C4R_USE_RTAUDIO_REPO}
)
message(STATUS "Fetching RtAudio sources ${C4R_USE_RTAUDIO_TAG} from git at ${C4R_USE_RTAUDIO_REPO}")
FetchContent_GetProperties(RtAudio_build)
if(NOT rtaudio_build_POPULATED)
    FetchContent_Populate(RtAudio_build)
    message(STATUS "Building RtAudio in ${rtaudio_build_BINARY_DIR} from sources in ${rtaudio_build_SOURCE_DIR}")
    add_subdirectory(${rtaudio_build_SOURCE_DIR} ${rtaudio_build_BINARY_DIR} EXCLUDE_FROM_ALL)
endif()
set_target_properties(rtaudio PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(cwAsio4RtAudioDriver MODULE)

target_include_directories(cwAsio4RtAudioDriver PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/../src)
set_target_properties(cwAsio4RtAudioDriver PROPERTIES POSITION_INDEPENDENT_CODE ON)
set_property(TARGET cwAsio4RtAudioDriver PROPERTY
    PUBLIC_HEADER
)

target_link_options(cwAsio4RtAudioDriver PRIVATE "-Wl,--version-script=${cwasio_build_SOURCE_DIR}/src/cwASIOdriver.map")

target_link_libraries(cwAsio4RtAudioDriver PRIVATE $<BUILD_INTERFACE:cwASIO::driver> $<BUILD_INTERFACE:rtaudio>)

target_sources(cwAsio4RtAudioDriver
PUBLIC
PRIVATE
    cwAsio4RtAudio.cpp
)
