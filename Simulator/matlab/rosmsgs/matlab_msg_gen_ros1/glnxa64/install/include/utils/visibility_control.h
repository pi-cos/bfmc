#ifndef UTILS__VISIBILITY_CONTROL_H_
#define UTILS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define UTILS_EXPORT __attribute__ ((dllexport))
    #define UTILS_IMPORT __attribute__ ((dllimport))
  #else
    #define UTILS_EXPORT __declspec(dllexport)
    #define UTILS_IMPORT __declspec(dllimport)
  #endif
  #ifdef UTILS_BUILDING_LIBRARY
    #define UTILS_PUBLIC UTILS_EXPORT
  #else
    #define UTILS_PUBLIC UTILS_IMPORT
  #endif
  #define UTILS_PUBLIC_TYPE UTILS_PUBLIC
  #define UTILS_LOCAL
#else
  #define UTILS_EXPORT __attribute__ ((visibility("default")))
  #define UTILS_IMPORT
  #if __GNUC__ >= 4
    #define UTILS_PUBLIC __attribute__ ((visibility("default")))
    #define UTILS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define UTILS_PUBLIC
    #define UTILS_LOCAL
  #endif
  #define UTILS_PUBLIC_TYPE
#endif
#endif  // UTILS__VISIBILITY_CONTROL_H_
// Generated 21-Nov-2022 16:14:41
// Copyright 2019-2020 The MathWorks, Inc.
