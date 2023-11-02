# CDDL HEADER START
#
# This file and its contents are supplied under the terms of the
# Common Development and Distribution License ("CDDL"), version 1.0.
# You may only use this file in accordance with the terms of version
# 1.0 of the CDDL.
#
# A full copy of the text of the CDDL should have accompanied this
# source.  A copy of the CDDL is also available via the Internet at
# http://www.illumos.org/license/CDDL.
#
# CDDL HEADER END

# Copyright 2022 Saso Kiselkov. All rights reserved.

cmake_minimum_required(VERSION 3.0)

project(opengpws)

option(LIBACFUTILS	"libacfutils source path")
if(${LIBACFUTILS} STREQUAL "OFF")
	message("Missing LIBACFUTILS option. Call cmake -DLIBACFUTILS=<path>")
	return()
endif()

option(NOAUDIO		"Set to 1 to disable audio support")

# Platform-specific variables
if(APPLE)
	set(PLAT_SHORT "mac64")
	set(PLAT_LONG "mac-64")
	set(PLAT_ONLY "mac")
	set(PLUGIN_BIN_OUTDIR "mac_x64")
elseif(WIN32)
	set(PLAT_SHORT "win64")
	set(PLAT_LONG "win-64")
	set(PLAT_ONLY "win")
	set(PLUGIN_BIN_OUTDIR "win_x64")
else()
	set(PLAT_SHORT "lin64")
	set(PLAT_LONG "linux-64")
	set(PLAT_ONLY "linux")
	set(PLUGIN_BIN_OUTDIR "lin_x64")
endif()

# Build path setup
file(GLOB GLEW_PREFIX "${LIBACFUTILS}/glew/glew-1.13.0-${PLAT_LONG}/install")
file(GLOB OPENAL_PREFIX
    "${LIBACFUTILS}/openal-soft/openal-soft-1.21.0-${PLAT_LONG}/build/install")
file(GLOB CAIRO_PREFIX "${LIBACFUTILS}/cairo/cairo-${PLAT_LONG}")
file(GLOB PIXMAN_PREFIX "${LIBACFUTILS}/cairo/pixman-${PLAT_LONG}")
file(GLOB LIBPNG_PREFIX "${LIBACFUTILS}/libpng/libpng-${PLAT_LONG}")
file(GLOB FREETYPE_PREFIX "${LIBACFUTILS}/freetype/freetype-${PLAT_LONG}")
file(GLOB OPUSFILE_PREFIX "${LIBACFUTILS}/opus/opusfile-${PLAT_LONG}")
file(GLOB LIBOGG_PREFIX "${LIBACFUTILS}/opus/libogg-${PLAT_LONG}/install")
file(GLOB OPENSSL_PREFIX "${LIBACFUTILS}/ssl/openssl-${PLAT_LONG}")
file(GLOB ICONV_PREFIX "${LIBACFUTILS}/libiconv/libiconv-${PLAT_LONG}")
file(GLOB LZMA_PREFIX "${LIBACFUTILS}/lzma/qmake/${PLAT_LONG}")
file(GLOB SHP_PREFIX "${LIBACFUTILS}/shapelib/libshape-${PLAT_LONG}")

file(GLOB SRC *.c)
file(GLOB HDR *.h ../api/opengpws/xplane_api.h)

set(CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELEASE} -DDEBUG -g")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -DDEBUG -O0 -g")

set(ALL_SRC ${SRC} ${HDR})
list(SORT ALL_SRC)

if(APPLE)
	add_executable(opengpws ${ALL_SRC})
else()
	add_library(opengpws SHARED ${ALL_SRC})
endif()

include_directories(opengpws PUBLIC
    "${LIBACFUTILS}/src"
    "${GLEW_PREFIX}/include"
    "..")

#compiler flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Werror --std=c99 \
    -Wno-unused-local-typedefs -Wno-missing-field-initializers \
    -Wno-deprecated")
if(!APPLE)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fno-gnu-unique")
endif()

add_definitions(-D_GNU_SOURCE)
add_definitions(-D_FILE_OFFSET_BITS=64)
add_definitions(-DDEBUG)
add_definitions("-DCHECK_RESULT_USED=__attribute__((warn_unused_result))")
if(${FAST_DEBUG})
	add_definitions(-DFAST_DEBUG=${FAST_DEBUG})
endif()
if(${NOAUDIO})
	add_definitions(-DNOAUDIO=1)
endif()

# Yes, we need to strip a trailing newline from command output. CMake is
# "smart" like that...
execute_process(COMMAND git rev-parse --short HEAD
    OUTPUT_VARIABLE PLUGIN_VERSION)
string(REGEX REPLACE "\n$" "" PLUGIN_VERSION "${PLUGIN_VERSION}")

add_definitions(-DPLUGIN_VERSION="${PLUGIN_VERSION}")
add_definitions(-DXPLM200=1 -DXPLM210=1 -DGLEW_BUILD=GLEW_STATIC)

if(APPLE)
	add_definitions(-DAPL=1 -DIBM=0 -DLIN=0)
	execute_process(COMMAND ${LIBACFUTILS}/pkg-config-deps mac-64 --cflags
	    OUTPUT_VARIABLE DEP_CFLAGS)
	execute_process(COMMAND ${LIBACFUTILS}/pkg-config-deps mac-64 --libs
	    OUTPUT_VARIABLE LIBACFUTILS_DEP_LDFLAGS)
	set_target_properties(opengpws PROPERTIES LINK_FLAGS
	    "${CMAKE_SHARED_LINKER_FLAGS} -fvisibility=hidden -bundle")
elseif(WIN32)
	add_definitions(-DAPL=0 -DIBM=1 -DLIN=0 -D_WIN32_WINNT=0x0600)
	execute_process(COMMAND ${LIBACFUTILS}/pkg-config-deps win-64 --cflags
	    OUTPUT_VARIABLE DEP_CFLAGS)
	execute_process(COMMAND ${LIBACFUTILS}/pkg-config-deps win-64 --libs
	    OUTPUT_VARIABLE LIBACFUTILS_DEP_LDFLAGS)
	set_target_properties(opengpws PROPERTIES LINK_FLAGS
	    "${CMAKE_SHARED_LINKER_FLAGS} -undefined_warning \
	    -fvisibility=hidden -static-libgcc -static-libstdc++")
else()
	add_definitions(-DAPL=0 -DIBM=0 -DLIN=1)
	execute_process(COMMAND ${LIBACFUTILS}/pkg-config-deps linux-64 --cflags
	    OUTPUT_VARIABLE DEP_CFLAGS)
	execute_process(COMMAND ${LIBACFUTILS}/pkg-config-deps linux-64 --libs
	    OUTPUT_VARIABLE LIBACFUTILS_DEP_LDFLAGS)
	set_target_properties(opengpws PROPERTIES LINK_FLAGS
	    "${CMAKE_SHARED_LINKER_FLAGS} -rdynamic -nodefaultlibs \
	    -undefined_warning -fPIC -fvisibility=hidden \
	    -static-libgcc -static-libstdc++")
endif()

string(REGEX REPLACE "\n$" "" DEP_CFLAGS "${DEP_CFLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${DEP_CFLAGS}")

#libraries
include_directories(opengpws PUBLIC
    "${LIBACFUTILS}/SDK/CHeaders/XPLM"
    "${LIBACFUTILS}/SDK/CHeaders/Widgets"
    "${LIBACFUTILS}/SDK/CHeaders/Wrappers"
    "${OPENAL_PREFIX}/include/AL"
    "${CMAKE_SOURCE_DIR}/../api")

# linking
# Careful: order matters in target_link_libraries. Dependencies must be
# ordered as first symbol callers and then as symbol providers (i.e.
# libpng16 depends on libz, so linking must be first: libpng, then libz).
link_libraries(m)
file(GLOB LIBACFUTILS_LIBRARY
    "${LIBACFUTILS}/qmake/${PLAT_SHORT}/libacfutils.a")
file(GLOB LIBPNG_LIBRARY "${LIBPNG_PREFIX}/lib/libpng16.a")
file(GLOB ZLIB_LIBRARY "${LIBACFUTILS}/zlib/zlib-${PLAT_LONG}/lib/libz.a")
file(GLOB OPUSFILE_LIBRARY  "${OPUSFILE_PREFIX}/lib/libopusfile.a")
file(GLOB OPUS_LIBRARY "${LIBACFUTILS}/opus/opus-${PLAT_LONG}/lib/libopus.a")
file(GLOB OGG_LIBRARY "${LIBOGG_PREFIX}/lib/libogg.a")
if(WIN32)
	file(GLOB OPENAL_LIBRARY "${OPENAL_PREFIX}/lib/libOpenAL32.a")
else()
	file(GLOB OPENAL_LIBRARY "${OPENAL_PREFIX}/lib/libopenal.a")
endif()
file(GLOB FREETYPE_LIBRARY "${FREETYPE_PREFIX}/lib/libfreetype.a")
file(GLOB CAIRO_LIBRARY "${CAIRO_PREFIX}/lib/libcairo.a")
file(GLOB PIXMAN_LIBRARY  "${PIXMAN_PREFIX}/lib/libpixman-1.a")
if(WIN32)
	file(GLOB GLEW_LIBRARY "${GLEW_PREFIX}/lib/libglew32mx.a")
elseif(APPLE)
	file(GLOB GLEW_LIBRARY "${GLEW_PREFIX}/lib/libGLEWmx.a")
else()
	file(GLOB GLEW_LIBRARY "${GLEW_PREFIX}/lib64/libGLEWmx.a")
endif()
file(GLOB CURL_LIBRARY "${LIBACFUTILS}/curl/libcurl-${PLAT_LONG}/lib/libcurl.a")
file(GLOB SSL_LIBRARY "${OPENSSL_PREFIX}/lib/libssl.a")
file(GLOB CRYPTO_LIBRARY "${OPENSSL_PREFIX}/lib/libcrypto.a")
file(GLOB LZMA_LIBRARY "${LZMA_PREFIX}/liblzma.a")
file(GLOB SHP_LIBRARY "${SHP_PREFIX}/lib/libshp.a")
file(GLOB ICONV_LIBRARY "${ICONV_PREFIX}/lib/libiconv.a")

if(APPLE)
	find_library(XPLM_FRAMEWORK XPLM "${LIBACFUTILS}/SDK/Libraries/Mac")
	find_library(XPWIDGETS_FRAMEWORK XPWidgets
	    "${LIBACFUTILS}/SDK/Libraries/Mac")
	find_library(OPENGL_FRAMEWORK OpenGL)
	find_library(AUDIOUNIT_FRAMEWORK AudioUnit)
	find_library(COREAUDIO_FRAMEWORK CoreAudio)
	find_library(COCOA_FRAMEWORK Cocoa)
	find_library(SYSTEM_CONFIGURATION_FRAMEWORK SystemConfiguration)
	set(EXTRA_PLATFORM_LIBS
	    ${XPLM_FRAMEWORK}
	    ${XPWIDGETS_FRAMEWORK}
	    ${OPENGL_FRAMEWORK}
	    ${COCOA_FRAMEWORK}
	    ${AUDIOUNIT_FRAMEWORK}
	    ${COREAUDIO_FRAMEWORK}
	    ${SYSTEM_CONFIGURATION_FRAMEWORK}
	    )
elseif(WIN32)
	find_library(XPLM_LIBRARY XPLM_64 "${LIBACFUTILS}/SDK/Libraries/Win")
	find_library(XPWIDGETS_LIBRARY XPWidgets_64
	    "${LIBACFUTILS}/SDK/Libraries/Win")
	set(EXTRA_PLATFORM_LIBS
	    ${XPLM_LIBRARY}
	    ${XPWIDGETS_LIBRARY}
	    "-lglu32"
	    "-lopengl32"
	    "-lws2_32"
	    "-lwinmm"
	    "-ldbghelp"
	    "-lpsapi"
	    "-lcrypt32"
	    "-lbcrypt"
	    )
else()
	set(EXTRA_PLATFORM_LIBS "")
endif()

target_link_libraries(opengpws
    ${LIBACFUTILS_LIBRARY}
    ${CURL_LIBRARY}
    ${SSL_LIBRARY}
    ${CRYPTO_LIBRARY}
    ${OPUSFILE_LIBRARY}
    ${OPUS_LIBRARY}
    ${OGG_LIBRARY}
    ${CAIRO_LIBRARY}
    ${PIXMAN_LIBRARY}
    ${FREETYPE_LIBRARY}
    ${LIBPNG_LIBRARY}
    ${GLEW_LIBRARY}
    ${SHP_LIBRARY}
    ${OPENAL_LIBRARY}
    ${ICONV_LIBRARY}
    ${ZLIB_LIBRARY}
    ${LZMA_LIBRARY}
    ${EXTRA_PLATFORM_LIBS}
    )

SET_TARGET_PROPERTIES(opengpws PROPERTIES PREFIX "")
SET_TARGET_PROPERTIES(opengpws PROPERTIES SUFFIX "")
set_target_properties(opengpws PROPERTIES LINKER_LANGUAGE CXX)

set_target_properties(opengpws PROPERTIES RUNTIME_OUTPUT_DIRECTORY
    "${CMAKE_SOURCE_DIR}/../OpenGPWS/${PLUGIN_BIN_OUTDIR}")
set_target_properties(opengpws PROPERTIES LIBRARY_OUTPUT_DIRECTORY
    "${CMAKE_SOURCE_DIR}/../OpenGPWS/${PLUGIN_BIN_OUTDIR}")
set_target_properties(opengpws PROPERTIES OUTPUT_NAME "OpenGPWS.xpl")
