set(CMAKE_CXX_COMPILER "/opt/odroid-x2/compiler/bin/arm-linux-gnueabihf-g++")
set(CMAKE_CXX_COMPILER_ARG1 "")
set(CMAKE_CXX_COMPILER_ID "GNU")
set(CMAKE_CXX_COMPILER_VERSION "4.8.3")
set(CMAKE_CXX_PLATFORM_ID "Linux")

set(CMAKE_AR "/opt/odroid-x2/compiler/bin/arm-linux-gnueabihf-ar")
set(CMAKE_RANLIB "/opt/odroid-x2/compiler/bin/arm-linux-gnueabihf-ranlib")
set(CMAKE_LINKER "/opt/odroid-x2/compiler/bin/arm-linux-gnueabihf-ld")
set(CMAKE_COMPILER_IS_GNUCXX 1)
set(CMAKE_CXX_COMPILER_LOADED 1)
set(CMAKE_CXX_COMPILER_WORKS TRUE)
set(CMAKE_CXX_ABI_COMPILED TRUE)
set(CMAKE_COMPILER_IS_MINGW )
set(CMAKE_COMPILER_IS_CYGWIN )
if(CMAKE_COMPILER_IS_CYGWIN)
  set(CYGWIN 1)
  set(UNIX 1)
endif()

set(CMAKE_CXX_COMPILER_ENV_VAR "CXX")

if(CMAKE_COMPILER_IS_MINGW)
  set(MINGW 1)
endif()
set(CMAKE_CXX_COMPILER_ID_RUN 1)
set(CMAKE_CXX_IGNORE_EXTENSIONS inl;h;hpp;HPP;H;o;O;obj;OBJ;def;DEF;rc;RC)
set(CMAKE_CXX_SOURCE_FILE_EXTENSIONS C;M;c++;cc;cpp;cxx;m;mm;CPP)
set(CMAKE_CXX_LINKER_PREFERENCE 30)
set(CMAKE_CXX_LINKER_PREFERENCE_PROPAGATES 1)

# Save compiler ABI information.
set(CMAKE_CXX_SIZEOF_DATA_PTR "4")
set(CMAKE_CXX_COMPILER_ABI "ELF")
set(CMAKE_CXX_LIBRARY_ARCHITECTURE "arm-linux-gnueabihf")

if(CMAKE_CXX_SIZEOF_DATA_PTR)
  set(CMAKE_SIZEOF_VOID_P "${CMAKE_CXX_SIZEOF_DATA_PTR}")
endif()

if(CMAKE_CXX_COMPILER_ABI)
  set(CMAKE_INTERNAL_PLATFORM_ABI "${CMAKE_CXX_COMPILER_ABI}")
endif()

if(CMAKE_CXX_LIBRARY_ARCHITECTURE)
  set(CMAKE_LIBRARY_ARCHITECTURE "arm-linux-gnueabihf")
endif()




set(CMAKE_CXX_IMPLICIT_LINK_LIBRARIES "stdc++;m;c")
set(CMAKE_CXX_IMPLICIT_LINK_DIRECTORIES "/opt/odroid-x2/sdk/usr/lib/arm-linux-gnueabihf/mesa-egl;/opt/odroid-x2/sdk/usr/lib/arm-linux-gnueabihf/mesa;/opt/odroid-x2/compiler/lib/gcc/arm-linux-gnueabihf/4.8.3;/opt/odroid-x2/compiler/lib/gcc/arm-linux-gnueabihf;/opt/odroid-x2/compiler/lib/gcc;/opt/odroid-x2/compiler/arm-linux-gnueabihf/lib;/opt/odroid-x2/compiler/arm-linux-gnueabihf/libc/lib/arm-linux-gnueabihf;/opt/odroid-x2/compiler/arm-linux-gnueabihf/libc/lib;/opt/odroid-x2/compiler/arm-linux-gnueabihf/libc/usr/lib/arm-linux-gnueabihf;/opt/odroid-x2/compiler/arm-linux-gnueabihf/libc/usr/lib")
set(CMAKE_CXX_IMPLICIT_LINK_FRAMEWORK_DIRECTORIES "")



