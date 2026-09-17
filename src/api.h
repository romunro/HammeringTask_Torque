#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define HammeringTask_Torque_DLLIMPORT __declspec(dllimport)
#  define HammeringTask_Torque_DLLEXPORT __declspec(dllexport)
#  define HammeringTask_Torque_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define HammeringTask_Torque_DLLIMPORT __attribute__((visibility("default")))
#    define HammeringTask_Torque_DLLEXPORT __attribute__((visibility("default")))
#    define HammeringTask_Torque_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define HammeringTask_Torque_DLLIMPORT
#    define HammeringTask_Torque_DLLEXPORT
#    define HammeringTask_Torque_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef HammeringTask_Torque_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define HammeringTask_Torque_DLLAPI
#  define HammeringTask_Torque_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef HammeringTask_Torque_EXPORTS
#    define HammeringTask_Torque_DLLAPI HammeringTask_Torque_DLLEXPORT
#  else
#    define HammeringTask_Torque_DLLAPI HammeringTask_Torque_DLLIMPORT
#  endif // HammeringTask_Torque_EXPORTS
#  define HammeringTask_Torque_LOCAL HammeringTask_Torque_DLLLOCAL
#endif // HammeringTask_Torque_STATIC