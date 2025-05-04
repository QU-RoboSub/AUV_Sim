#ifndef SIMULATION__VISIBILITY_CONTROL_H_
#define SIMULATION__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define SIMULATION_EXPORT __attribute__ ((dllexport))
    #define SIMULATION_IMPORT __attribute__ ((dllimport))
  #else
    #define SIMULATION_EXPORT __declspec(dllexport)
    #define SIMULATION_IMPORT __declspec(dllimport)
  #endif
  #ifdef SIMULATION_BUILDING_LIBRARY
    #define SIMULATION_PUBLIC SIMULATION_EXPORT
  #else
    #define SIMULATION_PUBLIC SIMULATION_IMPORT
  #endif
  #define SIMULATION_PUBLIC_TYPE SIMULATION_PUBLIC
  #define SIMULATION_LOCAL
#else
  #define SIMULATION_EXPORT __attribute__ ((visibility("default")))
  #define SIMULATION_IMPORT
  #if __GNUC__ >= 4
    #define SIMULATION_PUBLIC __attribute__ ((visibility("default")))
    #define SIMULATION_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define SIMULATION_PUBLIC
    #define SIMULATION_LOCAL
  #endif
  #define SIMULATION_PUBLIC_TYPE
#endif
#endif  // SIMULATION__VISIBILITY_CONTROL_H_
// Generated 03-May-2025 19:07:52
 