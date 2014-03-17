FIND_PATH(
    CRASH_INCLUDE_DIRS
    NAMES libcrash.h
    PATHS /usr/local/include/
          /usr/include/
)

FIND_LIBRARY(
    CRASH_LIBRARIES
    NAMES libcrash.so
    PATHS /usr/local/lib
          /usr/lib
          /usr/lib/arm-linux-gnueabihf/
)

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(CRASH DEFAULT_MSG CRASH_LIBRARIES CRASH_INCLUDE_DIRS)
MARK_AS_ADVANCED(CRASH_LIBRARIES CRASH_INCLUDE_DIRS)