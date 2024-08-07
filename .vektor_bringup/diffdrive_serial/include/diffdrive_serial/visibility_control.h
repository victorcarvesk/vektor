#ifndef ESP_HW_COMPONENT__VISIBILITY_CONTROL_H_
#define ESP_HW_COMPONENT__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ESP_HW_COMPONENT_EXPORT __attribute__ ((dllexport))
    #define ESP_HW_COMPONENT_IMPORT __attribute__ ((dllimport))
  #else
    #define ESP_HW_COMPONENT_EXPORT __declspec(dllexport)
    #define ESP_HW_COMPONENT_IMPORT __declspec(dllimport)
  #endif
  #ifdef ESP_HW_COMPONENT_BUILDING_LIBRARY
    #define ESP_HW_COMPONENT_PUBLIC ESP_HW_COMPONENT_EXPORT
  #else
    #define ESP_HW_COMPONENT_PUBLIC ESP_HW_COMPONENT_IMPORT
  #endif
  #define ESP_HW_COMPONENT_PUBLIC_TYPE ESP_HW_COMPONENT_PUBLIC
  #define ESP_HW_COMPONENT_LOCAL
#else
  #define ESP_HW_COMPONENT_EXPORT __attribute__ ((visibility("default")))
  #define ESP_HW_COMPONENT_IMPORT
  #if __GNUC__ >= 4
    #define ESP_HW_COMPONENT_PUBLIC __attribute__ ((visibility("default")))
    #define ESP_HW_COMPONENT_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ESP_HW_COMPONENT_PUBLIC
    #define ESP_HW_COMPONENT_LOCAL
  #endif
  #define ESP_HW_COMPONENT_PUBLIC_TYPE
#endif

#endif  // ESP_HW_COMPONENT__VISIBILITY_CONTROL_H_
