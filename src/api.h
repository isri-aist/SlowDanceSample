#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define SlowDance_DLLIMPORT __declspec(dllimport)
#  define SlowDance_DLLEXPORT __declspec(dllexport)
#  define SlowDance_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define SlowDance_DLLIMPORT __attribute__((visibility("default")))
#    define SlowDance_DLLEXPORT __attribute__((visibility("default")))
#    define SlowDance_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define SlowDance_DLLIMPORT
#    define SlowDance_DLLEXPORT
#    define SlowDance_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef SlowDance_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define SlowDance_DLLAPI
#  define SlowDance_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef SlowDance_EXPORTS
#    define SlowDance_DLLAPI SlowDance_DLLEXPORT
#  else
#    define SlowDance_DLLAPI SlowDance_DLLIMPORT
#  endif // SlowDance_EXPORTS
#  define SlowDance_LOCAL SlowDance_DLLLOCAL
#endif // SlowDance_STATIC