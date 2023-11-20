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

#ifndef GAWRON_PLANE_META__VISIBILITY_CONTROL_HPP_
#define GAWRON_PLANE_META__VISIBILITY_CONTROL_HPP_

////////////////////////////////////////////////////////////////////////////////
#if defined(__WIN32)
  #if defined(GAWRON_PLANE_META_BUILDING_DLL) || defined(GAWRON_PLANE_META_EXPORTS)
    #define GAWRON_PLANE_META_PUBLIC __declspec(dllexport)
    #define GAWRON_PLANE_META_LOCAL
  #else  // defined(GAWRON_PLANE_META_BUILDING_DLL) || defined(GAWRON_PLANE_META_EXPORTS)
    #define GAWRON_PLANE_META_PUBLIC __declspec(dllimport)
    #define GAWRON_PLANE_META_LOCAL
  #endif  // defined(GAWRON_PLANE_META_BUILDING_DLL) || defined(GAWRON_PLANE_META_EXPORTS)
#elif defined(__linux__)
  #define GAWRON_PLANE_META_PUBLIC __attribute__((visibility("default")))
  #define GAWRON_PLANE_META_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define GAWRON_PLANE_META_PUBLIC __attribute__((visibility("default")))
  #define GAWRON_PLANE_META_LOCAL __attribute__((visibility("hidden")))
#else
  #error "Unsupported Build Configuration"
#endif

#endif  // GAWRON_PLANE_META__VISIBILITY_CONTROL_HPP_
