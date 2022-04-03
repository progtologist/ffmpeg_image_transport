# - Find VA-API
# Find the VA-API includes and library
# This module defines
#  VAAPI_INCLUDE_DIR, where to find db.h, etc.
#  VAAPI_LIBRARIES, the libraries needed to use VA-API.
#  VAAPI_FOUND, If false, do not try to use VA-API.
# also defined, but not for general use are
#  VAAPI_LIBRARY, where to find the VA-API library.

FIND_PATH(VAAPI_INCLUDE_DIR 
  NAMES va/va.h
  PATHS /usr/local/include /usr/include
)

FIND_LIBRARY(VAX11_LIBRARY
  NAMES "va-x11"
  PATHS /usr/lib /usr/local/lib
)

FIND_LIBRARY(VA_LIBRARY
  NAMES "va"
  PATHS /usr/lib /usr/local/lib
)

SET(VAAPI_LIBRARIES ${VAX11_LIBRARY} ${VA_LIBRARY})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(VAAPI DEFAULT_MSG VAAPI_LIBRARIES VAAPI_INCLUDE_DIR)
mark_as_advanced(VAAPI_INCLUDE_DIR VAAPI_LIBRARIES)