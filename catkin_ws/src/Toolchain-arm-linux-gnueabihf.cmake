SET(CMAKE_SYSTEM_NAME Linux)
SET(CMAKE_SYSTEM_VERSION 1)

# specify the cross compiler
SET(CMAKE_C_COMPILER   /opt/odroid-x2/compiler/bin/arm-linux-gnueabihf-gcc)
SET(CMAKE_CXX_COMPILER /opt/odroid-x2/compiler/bin/arm-linux-gnueabihf-g++)

# Set the target architecture. This should ideally be self-detected (as it is
# with Ubuntu 12.04's arm-linux-gnueabihf-g++ 4.6 and Linaro's arm-...-g++ 4.8.2)
# but this fails for Ubuntu 14.04's arm-...-g++ 4.8.3.
set(CMAKE_LIBRARY_ARCHITECTURE arm-linux-gnueabihf)

# where is the target environment 
if (NOT CMAKE_FIND_ROOT_PATH)
	SET(CMAKE_FIND_ROOT_PATH "/opt/odroid-x2/sdk" CACHE INTERNAL "" FORCE)
else()
	GET_FILENAME_COMPONENT(CMAKE_FIND_ROOT_PATH "${CMAKE_FIND_ROOT_PATH}" ABSOLUTE)
endif ()
SET(CMAKE_FIND_ROOT_PATH "/opt/odroid-x2/sdk" CACHE INTERNAL "" FORCE)

## search for programs in the build host directories
SET(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
## for libraries and headers in the target directories
SET(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
SET(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
## search for packages only on the target system
#SET(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# set up pkg-config to look in the sysroot directory for the package configuration files
SET(ENV{PKG_CONFIG_DIR} "")
SET(ENV{PKG_CONFIG_LIBDIR} ${CMAKE_FIND_ROOT_PATH}/usr/lib/pkgconfig:${CMAKE_FIND_ROOT_PATH}/usr/share/pkgconfig:${CMAKE_FIND_ROOT_PATH}/usr/lib/arm-linux-gnueabihf/pkgconfig/)
SET(ENV{PKG_CONFIG_SYSROOT_DIR} ${CMAKE_FIND_ROOT_PATH})

#
#INCLUDE(${CMAKE_MODULE_PATH}/ForceAddFlags.cmake)
#FORCE_ADD_FLAGS(CMAKE_C_FLAGS -Wl,--allow-shlib-undefined --sysroot=${CMAKE_FIND_ROOT_PATH})
#FORCE_ADD_FLAGS(CMAKE_CXX_FLAGS -Wl,--allow-shlib-undefined --sysroot=${CMAKE_FIND_ROOT_PATH})
#add_definitions("-Wl,--allow-shlib-undefined --sysroot=${CMAKE_FIND_ROOT_PATH}")
SET(CMAKE_C_FLAGS "${CMAKE_CXX_FLAGS} --sysroot=${CMAKE_FIND_ROOT_PATH}")
#SET(CMAKE_CXX_FLAGS -Wl,--allow-shlib-undefined --sysroot=${CMAKE_FIND_ROOT_PATH})
#SET(CMAKE_LIN CXX_FLAGS -Wl,--allow-shlib-undefined --sysroot=${CMAKE_FIND_ROOT_PATH})
SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} --sysroot=${CMAKE_FIND_ROOT_PATH}" )
SET( CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} -Wl,--allow-shlib-undefined --sysroot=${CMAKE_FIND_ROOT_PATH}" )

# the gcc cross-compiler may not have the location of the C++ headers hard-coded, and
# when we change the sysroot it won't find them anymore (or rather expects them to be
# below the sysroot directory). To circumvent this, we add the correct location manually
# by checking how the cross-compiler was configured
if (CMAKE_COMPILER_IS_GNUCC)
	execute_process(COMMAND ${CMAKE_CXX_COMPILER} -v
					ERROR_VARIABLE GCC_INFO)
	string(REGEX REPLACE "^.*--with-gxx-include-dir=([^ ]*).*$" "\\1" GCC_CXX_PATH ${GCC_INFO})
	if (GCC_CXX_PATH)
		add_definitions("-isystem ${GCC_CXX_PATH} -isystem ${GCC_CXX_PATH}/arm-linux-gnueabihf")
	endif (GCC_CXX_PATH)
endif()

