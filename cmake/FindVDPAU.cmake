# - Find VDPAU
# Find the VDPAU includes and library
# This module defines
#  VDPAU_INCLUDE_DIR, where to find .h
#  VDPAU_LIBRARIES, the libraries needed to use VDPAU.
#  VDPAU_FOUND, If false, do not try to use VDPAU.
# also defined, but not for general use are
#  VDPAU_LIBRARY, where to find the VDPAU library.

FIND_PATH(VDPAU_INCLUDE_DIR 
  NAMES vdpau/vdpau.h vdpau/vdpau_x11.h
  PATHS /usr/local/include /usr/include
)

FIND_LIBRARY(VDPAU_LIBRARY
  NAMES "vdpau"
  PATHS /usr/lib /usr/local/lib
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(VDPAU DEFAULT_MSG VDPAU_LIBRARY VDPAU_INCLUDE_DIR)
set(VDPAU_LIBRARIES ${VDPAU_LIBRARY})
mark_as_advanced(VDPAU_INCLUDE_DIR VDPAU_LIBRARIES)