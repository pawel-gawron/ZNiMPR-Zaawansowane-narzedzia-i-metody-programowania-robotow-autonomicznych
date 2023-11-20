// Copyright 2022 Amadeusz Szymko
// Perception for Physical Interaction Laboratory at Poznan University of Technology
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


#ifndef NOISE_FILTER__VISIBILITY_CONTROL_HPP_
#define NOISE_FILTER__VISIBILITY_CONTROL_HPP_

////////////////////////////////////////////////////////////////////////////////
#if defined(__WIN32)
  #if defined(NOISE_FILTER_BUILDING_DLL) || defined(NOISE_FILTER_EXPORTS)
    #define NOISE_FILTER_PUBLIC __declspec(dllexport)
    #define NOISE_FILTER_LOCAL
  #else  // defined(NOISE_FILTER_BUILDING_DLL) || defined(NOISE_FILTER_EXPORTS)
    #define NOISE_FILTER_PUBLIC __declspec(dllimport)
    #define NOISE_FILTER_LOCAL
  #endif  // defined(NOISE_FILTER_BUILDING_DLL) || defined(NOISE_FILTER_EXPORTS)
#elif defined(__linux__)
  #define NOISE_FILTER_PUBLIC __attribute__((visibility("default")))
  #define NOISE_FILTER_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define NOISE_FILTER_PUBLIC __attribute__((visibility("default")))
  #define NOISE_FILTER_LOCAL __attribute__((visibility("hidden")))
#else
  #error "Unsupported Build Configuration"
#endif

#endif  // NOISE_FILTER__VISIBILITY_CONTROL_HPP_
