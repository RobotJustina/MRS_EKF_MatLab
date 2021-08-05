#ifndef CUSTOM_MSGS__VISIBILITY_CONTROL_H_
#define CUSTOM_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define CUSTOM_MSGS_EXPORT __attribute__ ((dllexport))
    #define CUSTOM_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define CUSTOM_MSGS_EXPORT __declspec(dllexport)
    #define CUSTOM_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef CUSTOM_MSGS_BUILDING_LIBRARY
    #define CUSTOM_MSGS_PUBLIC CUSTOM_MSGS_EXPORT
  #else
    #define CUSTOM_MSGS_PUBLIC CUSTOM_MSGS_IMPORT
  #endif
  #define CUSTOM_MSGS_PUBLIC_TYPE CUSTOM_MSGS_PUBLIC
  #define CUSTOM_MSGS_LOCAL
#else
  #define CUSTOM_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define CUSTOM_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define CUSTOM_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define CUSTOM_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define CUSTOM_MSGS_PUBLIC
    #define CUSTOM_MSGS_LOCAL
  #endif
  #define CUSTOM_MSGS_PUBLIC_TYPE
#endif
#endif  // CUSTOM_MSGS__VISIBILITY_CONTROL_H_
// Generated 03-Aug-2021 15:05:43
// Copyright 2019-2020 The MathWorks, Inc.
