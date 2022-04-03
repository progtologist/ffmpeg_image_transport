find_package(CUDA QUIET)
if (${CUDA_FOUND})
  message(STATUS "CUDA Found")
else()
  find_package(CUDAToolkit QUIET)
  if (${CUDAToolkit_FOUND})
    message(STATUS "CUDA Found")
    set(CUDA_FOUND TRUE)
  else()
    message(STATUS "CUDA Not Found")
  endif()
endif()

if (${CUDA_FOUND})
  set(FFMPEG-NVENC_trigger_build_dir ${CMAKE_BINARY_DIR}/force_ffmpeg_nvenc)

  file(MAKE_DIRECTORY ${FFMPEG-NVENC_trigger_build_dir} ${FFMPEG-NVENC_trigger_build_dir}/build)

  find_program(MAKE_EXE NAMES gmake nmake make)

  set(FFMPEG-NVENC_CMAKE_LIST_CONTENT "
cmake_minimum_required(VERSION 2.8)
project(force_ffmpeg_nvenc)

include(ExternalProject)
ExternalProject_add(ffmpeg_nvenc
        PREFIX ${CMAKE_CURRENT_BINARY_DIR}/ffmpeg_nvenc
        SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/ffmpeg_nvenc/src/ffmpeg_nvenc
        BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/ffmpeg_nvenc/src/ffmpeg_nvenc
        GIT_REPOSITORY https://git.videolan.org/git/ffmpeg/nv-codec-headers.git
        GIT_TAG n11.1.5.1
        CONFIGURE_COMMAND \"\"
        BUILD_COMMAND ${MAKE_EXE} -j $(nproc) --silent
        INSTALL_COMMAND ${MAKE_EXE} install PREFIX=${CMAKE_INSTALL_PREFIX}
        )

        add_custom_target(trigger_ffmpeg_nvenc)
        add_dependencies(trigger_ffmpeg_nvenc ffmpeg_nvenc)")

  file(WRITE ${FFMPEG-NVENC_trigger_build_dir}/CMakeLists.txt "${FFMPEG-NVENC_CMAKE_LIST_CONTENT}")

  execute_process(COMMAND ${CMAKE_COMMAND} ..
    WORKING_DIRECTORY ${FFMPEG-NVENC_trigger_build_dir}/build
  )

  execute_process(COMMAND ${CMAKE_COMMAND} --build . --
    WORKING_DIRECTORY ${FFMPEG-NVENC_trigger_build_dir}/build
  )

  find_path(FFMPEG_NVENC_INCLUDE_DIR ffnvcodec/nvEncodeAPI.h
    PATHS ${CMAKE_INSTALL_PREFIX}/include
    NO_DEFAULT_PATH
  )
  set(FFMPEG_NVENC_LIBRARIES ${CMAKE_INSTALL_PREFIX}/lib)

  mark_as_advanced(CUDA_FOUND FFMPEG_NVENC_INCLUDE_DIR FFMPEG_NVENC_LIBRARIES)
endif()
