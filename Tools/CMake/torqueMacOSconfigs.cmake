if(APPLE)
#detect Architecture
enable_language(OBJC)
enable_language(OBJCXX)
enable_language(CXX)

# minimum for multi arch build is 11.
set(CMAKE_OSX_DEPLOYMENT_TARGET "11" CACHE STRING "" FORCE)
set(CMAKE_OSX_ARCHITECTURES "x86_64;arm64" CACHE STRING "" FORCE)
set(CMAKE_XCODE_ATTRIBUTE_MACOSX_DEPLOYMENT_TARGET[arch=arm64] "11.0" CACHE STRING "arm 64 minimum deployment target" FORCE)

if(CMAKE_OSX_ARCHITECTURES MATCHES "((^|;|, )(arm64|arm64e|x86_64))+")
  set(CMAKE_C_SIZEOF_DATA_PTR 8)
  set(CMAKE_CXX_SIZEOF_DATA_PTR 8)
  if(CMAKE_OSX_ARCHITECTURES MATCHES "((^|;|, )(arm64|arm64e))+")
    set(CMAKE_SYSTEM_PROCESSOR "aarch64")
  else()
    set(CMAKE_SYSTEM_PROCESSOR "x86_64")
  endif()
else()
  set(CMAKE_C_SIZEOF_DATA_PTR 4)
  set(CMAKE_CXX_SIZEOF_DATA_PTR 4)
  set(CMAKE_SYSTEM_PROCESSOR "arm")
endif()

# Enable codesigning with secure timestamp when not in Debug configuration (required for Notarization)
set(CMAKE_XCODE_ATTRIBUTE_OTHER_CODE_SIGN_FLAGS[variant=Release] "--timestamp")
set(CMAKE_XCODE_ATTRIBUTE_OTHER_CODE_SIGN_FLAGS[variant=RelWithDebInfo] "--timestamp")

# Enable codesigning with hardened runtime option when not in Debug configuration (required for Notarization)
#set(CMAKE_XCODE_ATTRIBUTE_ENABLE_HARDENED_RUNTIME[variant=Release] YES)
#set(CMAKE_XCODE_ATTRIBUTE_ENABLE_HARDENED_RUNTIME[variant=RelWithDebInfo] YES)

# Disable injection of Xcode's base entitlements used for debugging when not in Debug configuration (required for
# Notarization)
set(CMAKE_XCODE_ATTRIBUTE_CODE_SIGN_INJECT_BASE_ENTITLEMENTS[variant=Release] NO)
set(CMAKE_XCODE_ATTRIBUTE_CODE_SIGN_INJECT_BASE_ENTITLEMENTS[variant=RelWithDebInfo] NO)

set(_release_configs RelWithDebInfo Release)
if(CMAKE_BUILD_TYPE IN_LIST _release_configs)
  add_link_options(LINKER:-dead_strip)
endif()

set(CMAKE_THREAD_LIBS_INIT "-lpthread")
set(CMAKE_HAVE_THREADS_LIBRARY 1)
set(CMAKE_USE_WIN32_THREADS_INIT 0)
set(CMAKE_USE_PTHREADS_INIT 1)
set(THREADS_PREFER_PTHREAD_FLAG ON)

set(CMAKE_MACOSX_RPATH 1)
set(CMAKE_INSTALL_RPATH "@executable_path/../Frameworks")

endif(APPLE)