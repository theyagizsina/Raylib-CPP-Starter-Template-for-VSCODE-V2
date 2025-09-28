# - Try to find raylib
# Once done this will define
#  raylib_FOUND - system has raylib
#  raylib_INCLUDE_DIRS - the raylib include directories
#  raylib_LIBRARIES - the libraries needed to use raylib
#
# Usage:
#   set(RAYLIB_ROOT "path/to/raylib") (optional)
#   find_package(raylib REQUIRED)

if (NOT DEFINED RAYLIB_ROOT)
    set(RAYLIB_ROOT "" CACHE PATH "Root directory for a local raylib build (containing src/)")
endif()

set(_raylib_hint_paths)
if (RAYLIB_ROOT)
    list(APPEND _raylib_hint_paths
        "${RAYLIB_ROOT}"
        "${RAYLIB_ROOT}/src"
        "${RAYLIB_ROOT}/lib"
        "${RAYLIB_ROOT}/lib64"
        "${RAYLIB_ROOT}/lib/win64"
        "${RAYLIB_ROOT}/lib/mingw"
    )
endif()

find_path(raylib_INCLUDE_DIR
    NAMES raylib.h
    HINTS ${_raylib_hint_paths}
    PATH_SUFFIXES include src
)

if (NOT raylib_INCLUDE_DIR AND RAYLIB_ROOT)
    # Fallback to canonical path inside RAYLIB_ROOT
    if (EXISTS "${RAYLIB_ROOT}/src/raylib.h")
        set(raylib_INCLUDE_DIR "${RAYLIB_ROOT}/src")
    endif()
endif()

set(_raylib_library_names raylib libraylib.a libraylib)
if (WIN32)
    list(APPEND _raylib_library_names raylib.lib)
endif()

find_library(raylib_LIBRARY
    NAMES ${_raylib_library_names}
    HINTS ${_raylib_hint_paths}
    PATH_SUFFIXES lib lib64 lib/win64 lib/mingw src
)

if (NOT raylib_LIBRARY AND RAYLIB_ROOT)
    # Common layout for source builds
    if (EXISTS "${RAYLIB_ROOT}/src/libraylib.a")
        set(raylib_LIBRARY "${RAYLIB_ROOT}/src/libraylib.a")
    elseif (EXISTS "${RAYLIB_ROOT}/lib/libraylib.a")
        set(raylib_LIBRARY "${RAYLIB_ROOT}/lib/libraylib.a")
    endif()
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(raylib
    REQUIRED_VARS raylib_LIBRARY raylib_INCLUDE_DIR
)

if (raylib_FOUND)
    set(raylib_INCLUDE_DIRS "${raylib_INCLUDE_DIR}")

    if (EXISTS "${raylib_INCLUDE_DIR}/external")
        list(APPEND raylib_INCLUDE_DIRS "${raylib_INCLUDE_DIR}/external")
    elseif (RAYLIB_ROOT AND EXISTS "${RAYLIB_ROOT}/src/external")
        list(APPEND raylib_INCLUDE_DIRS "${RAYLIB_ROOT}/src/external")
    endif()

    set(raylib_LIBRARIES "${raylib_LIBRARY}")

    if (NOT TARGET raylib::raylib)
        add_library(raylib::raylib UNKNOWN IMPORTED)
        set_target_properties(raylib::raylib PROPERTIES
            IMPORTED_LOCATION "${raylib_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES "${raylib_INCLUDE_DIRS}"
        )

        if (WIN32)
            set(_raylib_system_libs winmm gdi32 opengl32)
        elseif (APPLE)
            set(_raylib_system_libs "-framework Cocoa" "-framework IOKit" "-framework CoreVideo" "-framework OpenGL" "-framework OpenAL")
        else()
            set(_raylib_system_libs pthread m dl)
        endif()

        if (_raylib_system_libs)
            set_property(TARGET raylib::raylib PROPERTY INTERFACE_LINK_LIBRARIES "${_raylib_system_libs}")
        endif()
    endif()
endif()

mark_as_advanced(raylib_INCLUDE_DIR raylib_LIBRARY)
