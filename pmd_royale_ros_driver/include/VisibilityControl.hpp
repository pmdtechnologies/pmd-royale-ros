/****************************************************************************\
* Copyright (C) 2023 pmdtechnologies ag
*
* THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
* KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
* PARTICULAR PURPOSE.
*
\****************************************************************************/

#ifndef __PMD_ROYALE_ROS_DRIVER__VISIBILITY_CONTROL_H__
#define __PMD_ROYALE_ROS_DRIVER__VISIBILITY_CONTROL_H__

#ifdef __cplusplus
extern "C" {
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define PMD_ROYALE_ROS_DRIVER_EXPORT __attribute__((dllexport))
#define PMD_ROYALE_ROS_DRIVER_IMPORT __attribute__((dllimport))
#else
#define PMD_ROYALE_ROS_DRIVER_EXPORT __declspec(dllexport)
#define PMD_ROYALE_ROS_DRIVER_IMPORT __declspec(dllimport)
#endif
#ifdef PMD_ROYALE_ROS_DRIVER_BUILDING_DLL
#define PMD_ROYALE_ROS_DRIVER_PUBLIC PMD_ROYALE_ROS_DRIVER_EXPORT
#else
#define PMD_ROYALE_ROS_DRIVER_PUBLIC PMD_ROYALE_ROS_DRIVER_IMPORT
#endif
#define PMD_ROYALE_ROS_DRIVER_PUBLIC_TYPE PMD_ROYALE_ROS_DRIVER_PUBLIC
#define PMD_ROYALE_ROS_DRIVER_LOCAL
#else
#define PMD_ROYALE_ROS_DRIVER_EXPORT __attribute__((visibility("default")))
#define PMD_ROYALE_ROS_DRIVER_IMPORT
#if __GNUC__ >= 4
#define PMD_ROYALE_ROS_DRIVER_PUBLIC __attribute__((visibility("default")))
#define PMD_ROYALE_ROS_DRIVER_LOCAL __attribute__((visibility("hidden")))
#else
#define PMD_ROYALE_ROS_DRIVER_PUBLIC
#define PMD_ROYALE_ROS_DRIVER_LOCAL
#endif
#define PMD_ROYALE_ROS_DRIVER_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif // __PMD_ROYALE_ROS_DRIVER__VISIBILITY_CONTROL_H__
