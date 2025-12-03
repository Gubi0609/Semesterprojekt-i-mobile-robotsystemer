cmake_minimum_required(VERSION 3.10)

# Project name and version
project(AudioCommProtocol VERSION 1.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Build output directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/BUILD)

# Options for building different components
option(BUILD_DESKTOP "Build desktop/laptop programs (SRC/)" ON)
option(BUILD_PI "Build Raspberry Pi programs (src/)" OFF)
option(BUILD_TESTS "Build test programs" OFF)
option(BUILD_EXAMPLES "Build library examples" OFF)

# ============================================================================
# Find Required Libraries
# ============================================================================

# PortAudio - required for all audio programs
find_package(PkgConfig REQUIRED)
pkg_check_modules(PORTAUDIO REQUIRED portaudio-2.0)

# FFTW3 - required for frequency detection
find_library(FFTW3_LIBRARY fftw3 REQUIRED)

# SQLite3 - required for desktop programs only
if(BUILD_DESKTOP)
	find_package(SQLite3 REQUIRED)
endif()

# ROS - required for rb3_node_cpp only
if(BUILD_PI)
	find_package(catkin QUIET COMPONENTS
		roscpp
		std_msgs
		geometry_msgs
	)
	if(catkin_FOUND)
		catkin_package()
	endif()
endif()

# ============================================================================
# Include Directories
# ============================================================================

include_directories(
	${CMAKE_SOURCE_DIR}/INCLUDE
	${CMAKE_SOURCE_DIR}/LIB
	${CMAKE_SOURCE_DIR}/DATABASE
	${PORTAUDIO_INCLUDE_DIRS}
)

if(catkin_FOUND)
	include_directories(${catkin_INCLUDE_DIRS})
endif()

# ============================================================================
# Library Source Files
# ============================================================================

set(LIB_SOURCES
	LIB/audio_comm.cpp
	LIB/tone_generator_lib.cpp
	LIB/frequency_detector_lib.cpp
	LIB/audio_transmitter_lib.cpp
	LIB/audio_receiver_lib.cpp
)

set(DATABASE_SOURCES
	DATABASE/Database.cpp
	DATABASE/Logger.cpp
)

set(COMMON_SOURCES
	SRC/CRC.cpp
	SRC/command_protocol.cpp
)

# ============================================================================
# Desktop/Laptop Programs (SRC/)
# ============================================================================

if(BUILD_DESKTOP)
	message(STATUS "Building desktop programs (SRC/)")

	# Sender program (main.cpp)
	add_executable(sender
		SRC/main.cpp
		SRC/UI.cpp
		${COMMON_SOURCES}
		${LIB_SOURCES}
		${DATABASE_SOURCES}
	)
	target_link_libraries(sender
		${PORTAUDIO_LIBRARIES}
		${FFTW3_LIBRARY}
		SQLite::SQLite3
		m
		pthread
	)

	# Protocol sender
	add_executable(protocol_sender
		SRC/protocol_sender.cpp
		${COMMON_SOURCES}
		${LIB_SOURCES}
	)
	target_link_libraries(protocol_sender
		${PORTAUDIO_LIBRARIES}
		${FFTW3_LIBRARY}
		m
		pthread
	)

	message(STATUS "Desktop programs: sender, protocol_sender")
endif()

# ============================================================================
# Raspberry Pi Programs (src/)
# ============================================================================

if(BUILD_PI)
	message(STATUS "Building Raspberry Pi programs (src/)")

	# Protocol receiver
	add_executable(protocol_receiver
		src/protocol_receiver.cpp
		${COMMON_SOURCES}
		${LIB_SOURCES}
	)
	target_link_libraries(protocol_receiver
		${PORTAUDIO_LIBRARIES}
		${FFTW3_LIBRARY}
		m
		pthread
	)

	# Velocity provider library
	add_library(velocityProvider STATIC
		src/velocityProvider.cpp
	)

	# ROS node (if ROS is available)
	if(catkin_FOUND)
		add_executable(rb3_node_cpp
			src/rb3_node_cpp.cpp
			src/velocityProvider.cpp
			${COMMON_SOURCES}
			${LIB_SOURCES}
		)
		target_link_libraries(rb3_node_cpp
			${catkin_LIBRARIES}
			${PORTAUDIO_LIBRARIES}
			${FFTW3_LIBRARY}
			m
			pthread
		)
		message(STATUS "ROS found - building rb3_node_cpp")
	else()
		message(WARNING "ROS not found - skipping rb3_node_cpp")
	endif()

	message(STATUS "Pi programs: protocol_receiver")
endif()

# ============================================================================
# Test Programs (TESTS/)
# ============================================================================

if(BUILD_TESTS)
	message(STATUS "Building test programs (TESTS/)")

	# Frequency tests
	add_executable(frequency_test
		TESTS/frequency_test.cpp
		${COMMON_SOURCES}
		${LIB_SOURCES}
	)
	target_link_libraries(frequency_test
		${PORTAUDIO_LIBRARIES}
		${FFTW3_LIBRARY}
		m
		pthread
	)

	add_executable(frequency_test_single
		TESTS/frequency_test_single.cpp
		${COMMON_SOURCES}
		${LIB_SOURCES}
	)
	target_link_libraries(frequency_test_single
		${PORTAUDIO_LIBRARIES}
		${FFTW3_LIBRARY}
		m
		pthread
	)

	add_executable(frequency_transmitter_test
		TESTS/frequency_transmitter_test.cpp
		${COMMON_SOURCES}
		${LIB_SOURCES}
	)
	target_link_libraries(frequency_transmitter_test
		${PORTAUDIO_LIBRARIES}
		m
		pthread
	)

	# Chord tests
	add_executable(chord_diagnostic_test
		TESTS/chord_diagnostic_test.cpp
		${COMMON_SOURCES}
		${LIB_SOURCES}
	)
	target_link_libraries(chord_diagnostic_test
		${PORTAUDIO_LIBRARIES}
		${FFTW3_LIBRARY}
		m
		pthread
	)

	add_executable(chord_transmitter_test
		TESTS/chord_transmitter_test.cpp
		${COMMON_SOURCES}
		${LIB_SOURCES}
	)
	target_link_libraries(chord_transmitter_test
		${PORTAUDIO_LIBRARIES}
		m
		pthread
	)

	# WAV recorder
	add_executable(wav_recorder
		TESTS/simple_wav_recorder.cpp
		${LIB_SOURCES}
	)
	target_link_libraries(wav_recorder
		${PORTAUDIO_LIBRARIES}
		m
		pthread
	)

	# Protocol test
	add_executable(protocol_test
		TESTS/protocol_test.cpp
		${COMMON_SOURCES}
	)
	target_link_libraries(protocol_test
		m
		pthread
	)

	message(STATUS "Test programs: frequency_test, chord_diagnostic_test, wav_recorder, etc.")
endif()

# ============================================================================
# Library Examples (LIB/examples/)
# ============================================================================

if(BUILD_EXAMPLES)
	message(STATUS "Building library examples (LIB/examples/)")

	add_executable(chord_to_bits
		LIB/examples/chord_to_bits.cpp
		${LIB_SOURCES}
	)
	target_link_libraries(chord_to_bits
		${PORTAUDIO_LIBRARIES}
		${FFTW3_LIBRARY}
		m
		pthread
	)

	add_executable(onlytones
		LIB/examples/onlytones.cpp
		${LIB_SOURCES}
	)
	target_link_libraries(onlytones
		${PORTAUDIO_LIBRARIES}
		m
		pthread
	)

	add_executable(onlydetection
		LIB/examples/onlydection.cpp
		${LIB_SOURCES}
	)
	target_link_libraries(onlydetection
		${PORTAUDIO_LIBRARIES}
		${FFTW3_LIBRARY}
		m
		pthread
	)

	add_executable(tones_and_detection
		LIB/examples/tones_and_dection.cpp
		${LIB_SOURCES}
	)
	target_link_libraries(tones_and_detection
		${PORTAUDIO_LIBRARIES}
		${FFTW3_LIBRARY}
		m
		pthread
	)

	add_executable(crc_full_test
		LIB/examples/crc_chord_transmission.cpp
		${COMMON_SOURCES}
		${LIB_SOURCES}
	)
	target_link_libraries(crc_full_test
		${PORTAUDIO_LIBRARIES}
		${FFTW3_LIBRARY}
		m
		pthread
	)

	message(STATUS "Examples: chord_to_bits, onlytones, onlydetection, etc.")
endif()

# ============================================================================
# Print Configuration Summary
# ============================================================================

message(STATUS "")
message(STATUS "========================================")
message(STATUS "Audio Communication Protocol - Configuration")
message(STATUS "========================================")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Output directory: ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
message(STATUS "")
message(STATUS "Build Options:")
message(STATUS "  BUILD_DESKTOP  = ${BUILD_DESKTOP}")
message(STATUS "  BUILD_PI       = ${BUILD_PI}")
message(STATUS "  BUILD_TESTS    = ${BUILD_TESTS}")
message(STATUS "  BUILD_EXAMPLES = ${BUILD_EXAMPLES}")
message(STATUS "")
message(STATUS "Dependencies:")
message(STATUS "  PortAudio found: ${PORTAUDIO_FOUND}")
message(STATUS "  FFTW3 found: ${FFTW3_LIBRARY}")
if(BUILD_DESKTOP)
	message(STATUS "  SQLite3 found: ${SQLite3_FOUND}")
endif()
if(catkin_FOUND)
	message(STATUS "  ROS (catkin) found: YES")
else()
	message(STATUS "  ROS (catkin) found: NO")
endif()
message(STATUS "========================================")
message(STATUS "")
