// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#ifndef RCL_LIFECYCLE__VERSION_H_
#define RCL_LIFECYCLE__VERSION_H_

/// \def RCL_LIFECYCLE_VERSION_MAJOR
/// Defines RCL_LIFECYCLE major version number
#define RCL_LIFECYCLE_VERSION_MAJOR (9)

/// \def RCL_LIFECYCLE_VERSION_MINOR
/// Defines RCL_LIFECYCLE minor version number
#define RCL_LIFECYCLE_VERSION_MINOR (2)

/// \def RCL_LIFECYCLE_VERSION_PATCH
/// Defines RCL_LIFECYCLE version patch number
#define RCL_LIFECYCLE_VERSION_PATCH (7)

/// \def RCL_LIFECYCLE_VERSION_STR
/// Defines RCL_LIFECYCLE version string
#define RCL_LIFECYCLE_VERSION_STR "9.2.7"

/// \def RCL_LIFECYCLE_VERSION_GTE
/// Defines a macro to check whether the version of RCL_LIFECYCLE is greater than or equal to
/// the given version triple.
#define RCL_LIFECYCLE_VERSION_GTE(major, minor, patch) ( \
     (major < RCL_LIFECYCLE_VERSION_MAJOR) ? true \
     : ((major > RCL_LIFECYCLE_VERSION_MAJOR) ? false \
     : ((minor < RCL_LIFECYCLE_VERSION_MINOR) ? true \
     : ((minor > RCL_LIFECYCLE_VERSION_MINOR) ? false \
     : ((patch < RCL_LIFECYCLE_VERSION_PATCH) ? true \
     : ((patch > RCL_LIFECYCLE_VERSION_PATCH) ? false \
     : true))))))

#endif  // RCL_LIFECYCLE__VERSION_H_
