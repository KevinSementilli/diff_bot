#ifndef DIFF_BOT__VISIBILITY_CONTROL_H_
#define DIFF_BOT__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define DIFF_BOT_EXPORT __attribute__((dllexport))
#define DIFF_BOT_IMPORT __attribute__((dllimport))
#else
#define DIFF_BOT_EXPORT __declspec(dllexport)
#define DIFF_BOT_IMPORT __declspec(dllimport)
#endif
#ifdef DIFF_BOT_BUILDING_DLL
#define DIFF_BOT_PUBLIC DIFF_BOT_EXPORT
#else
#define DIFF_BOT_PUBLIC DIFF_BOT_IMPORT
#endif
#define DIFF_BOT_PUBLIC_TYPE DIFF_BOT_PUBLIC
#define DIFF_BOT_LOCAL
#else
#define DIFF_BOT_EXPORT __attribute__((visibility("default")))
#define DIFF_BOT_IMPORT
#if __GNUC__ >= 4
#define DIFF_BOT_PUBLIC __attribute__((visibility("default")))
#define DIFF_BOT_LOCAL __attribute__((visibility("hidden")))
#else
#define DIFF_BOT_PUBLIC
#define DIFF_BOT_LOCAL
#endif
#define DIFF_BOT_PUBLIC_TYPE
#endif

#endif  // DIFF_BOT__VISIBILITY_CONTROL_H_