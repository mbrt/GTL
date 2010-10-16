CMAKE_MINIMUM_REQUIRED (VERSION 2.6)
PROJECT ("Graph Template Library")
INCLUDE(CTest)

# The CMake build system is only used to perform some cheks on the gtl template
# library. If you only want to use the library you may prefer to copy only the
# header files (situated in ./src). You can use the test files as exmples for
# the library.
# NOTE: the boost library is used to make some checks in the test files. If
# you don't have it don't worry: the gtl library works only with the standard
# library, you don't have to install the boost library to use it.

# Here I assume that boost headers are installed in a self contained place.
# Please set the BOOST_INCLUDE_DIR to your local path (if it isn't the standard
# /usr/include/boost ).
SET (BOOST_INCLUDE_DIR "/usr/include")

INCLUDE_DIRECTORIES (
  "${CMAKE_SOURCE_DIR}"
  "${BOOST_INCLUDE_DIR}"
  )

ADD_SUBDIRECTORY(test)
