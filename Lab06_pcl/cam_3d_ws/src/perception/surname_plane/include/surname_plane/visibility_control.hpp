// Copyright 2023 Pawel_Gawron
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SURNAME_PLANE__VISIBILITY_CONTROL_HPP_
#define SURNAME_PLANE__VISIBILITY_CONTROL_HPP_

////////////////////////////////////////////////////////////////////////////////
#if defined(__WIN32)
  #if defined(SURNAME_PLANE_BUILDING_DLL) || defined(SURNAME_PLANE_EXPORTS)
    #define SURNAME_PLANE_PUBLIC __declspec(dllexport)
    #define SURNAME_PLANE_LOCAL
  #else  // defined(SURNAME_PLANE_BUILDING_DLL) || defined(SURNAME_PLANE_EXPORTS)
    #define SURNAME_PLANE_PUBLIC __declspec(dllimport)
    #define SURNAME_PLANE_LOCAL
  #endif  // defined(SURNAME_PLANE_BUILDING_DLL) || defined(SURNAME_PLANE_EXPORTS)
#elif defined(__linux__)
  #define SURNAME_PLANE_PUBLIC __attribute__((visibility("default")))
  #define SURNAME_PLANE_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define SURNAME_PLANE_PUBLIC __attribute__((visibility("default")))
  #define SURNAME_PLANE_LOCAL __attribute__((visibility("hidden")))
#else
  #error "Unsupported Build Configuration"
#endif

#endif  // SURNAME_PLANE__VISIBILITY_CONTROL_HPP_
