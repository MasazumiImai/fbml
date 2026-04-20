// Copyright (c) 2026 Masazumi Imai
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef FBML__VISIBILITY_CONTROL_H_
#define FBML__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define FBML_EXPORT __attribute__((dllexport))
#define FBML_IMPORT __attribute__((dllimport))
#else
#define FBML_EXPORT __declspec(dllexport)
#define FBML_IMPORT __declspec(dllimport)
#endif
#ifdef FBML_BUILDING_LIBRARY
#define FBML_PUBLIC FBML_EXPORT
#else
#define FBML_PUBLIC FBML_IMPORT
#endif
#define FBML_PUBLIC_TYPE FBML_PUBLIC
#define FBML_LOCAL
#else
#define FBML_EXPORT __attribute__((visibility("default")))
#define FBML_IMPORT
#if __GNUC__ >= 4
#define FBML_PUBLIC __attribute__((visibility("default")))
#define FBML_LOCAL __attribute__((visibility("hidden")))
#else
#define FBML_PUBLIC
#define FBML_LOCAL
#endif
#define FBML_PUBLIC_TYPE
#endif

#endif  // FBML__VISIBILITY_CONTROL_H_
