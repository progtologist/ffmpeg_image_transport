set(FFMPEG_trigger_build_dir ${CMAKE_BINARY_DIR}/force_ffmpeg)

file(MAKE_DIRECTORY ${FFMPEG_trigger_build_dir} ${FFMPEG_trigger_build_dir}/build)

find_program(MAKE_EXE NAMES gmake nmake make)

set(FFMPEG_ARGS "--enable-shared --disable-static \
  --disable-ffmpeg --disable-ffplay --disable-ffprobe \
  --disable-doc --disable-manpages --disable-postproc \
  --disable-avfilter --disable-alsa --disable-lzma \
  --disable-xlib --disable-sdl2 --disable-network \
  --enable-libaom --enable-libvpx --enable-libx264 \
  --enable-libx265 --enable-gpl --enable-gnutls \
  --enable-nonfree")

include(cmake/FindVDPAU.cmake)
if (${VDPAU_FOUND})
  set(FFMPEG_ARGS "${FFMPEG_ARGS} --enable-vdpau")
else()
  set(FFMPEG_ARGS "${FFMPEG_ARGS} --disable-vdpau")
endif()

include(cmake/FindVAAPI.cmake)
if (${VAAPI_FOUND})
  set(FFMPEG_ARGS "${FFMPEG_ARGS} --enable-vaapi")
else()
  set(FFMPEG_ARGS "${FFMPEG_ARGS} --disable-vaapi")
endif()

include(cmake/BuildFFMPEG-NVENC.cmake)
if (${CUDA_FOUND})
  message(STATUS "FFMPEG_NVENC_INCLUDE_DIR ${FFMPEG_NVENC_INCLUDE_DIR}")
  set(FFMPEG_ARGS "${FFMPEG_ARGS} --enable-cuda-nvcc --enable-libnpp \
  --extra-cflags=-I/usr/local/cuda/include \
  --extra-ldflags=-L/usr/local/cuda/lib64")
  set(FFMPEG_PREARGS "PKG_CONFIG_PATH=${FFMPEG_NVENC_LIBRARIES}/pkgconfig")
  message(STATUS "ARGS ARE ${FFMPEG_ARGS}")
  message(STATUS "PREARGS ARE ${FFMPEG_PREARGS}")
else()
  set(FFMPEG_PREARGS "")
endif()

set(FFMPEG_CMAKE_LIST_CONTENT "
cmake_minimum_required(VERSION 2.8)
project(force_ffmpeg)

include(ExternalProject)
ExternalProject_add(ffmpeg
        PREFIX ${CMAKE_CURRENT_BINARY_DIR}/ffmpeg
        SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/ffmpeg/src/ffmpeg
        BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/ffmpeg/src/ffmpeg
        GIT_REPOSITORY https://git.ffmpeg.org/ffmpeg.git
        GIT_TAG n4.2.4
        PATCH_COMMAND sed -i \"s/-gencode arch=compute_30,code=sm_30 -O2/-gencode arch=compute_60,code=sm_60 -O2/g\" ${CMAKE_CURRENT_BINARY_DIR}/ffmpeg/src/ffmpeg/configure
        CONFIGURE_COMMAND ${FFMPEG_PREARGS} ./configure --prefix=${CMAKE_INSTALL_PREFIX} ${FFMPEG_ARGS}
        BUILD_COMMAND ${MAKE_EXE} -j $(nproc) --silent
        INSTALL_COMMAND ${MAKE_EXE} install
        )

        add_custom_target(trigger_ffmpeg)
        add_dependencies(trigger_ffmpeg ffmpeg)")

file(WRITE ${FFMPEG_trigger_build_dir}/CMakeLists.txt "${FFMPEG_CMAKE_LIST_CONTENT}")

execute_process(COMMAND ${CMAKE_COMMAND} ..
  WORKING_DIRECTORY ${FFMPEG_trigger_build_dir}/build
)

execute_process(COMMAND ${CMAKE_COMMAND} --build . --
  WORKING_DIRECTORY ${FFMPEG_trigger_build_dir}/build
)

find_path(FFMPEG_AVCODEC_INCLUDE_DIR libavcodec/avcodec.h
  PATHS ${CMAKE_INSTALL_PREFIX}/include
  NO_DEFAULT_PATH)
find_library(FFMPEG_AVCODEC_LIBRARY avcodec
  PATHS ${CMAKE_INSTALL_PREFIX}/lib
  NO_DEFAULT_PATH)
find_library(FFMPEG_SWSCALE_LIBRARY swscale
  PATHS ${CMAKE_INSTALL_PREFIX}/lib
  NO_DEFAULT_PATH)

list(APPEND FFMPEG_INCLUDE_DIR ${FFMPEG_AVCODEC_INCLUDE_DIR})
list(APPEND FFMPEG_LIBRARIES ${FFMPEG_AVCODEC_LIBRARY})
list(APPEND FFMPEG_LIBRARIES ${FFMPEG_SWSCALE_LIBRARY})

mark_as_advanced(FFMPEG_INCLUDE_DIR FFMPEG_LIBRARIES)
