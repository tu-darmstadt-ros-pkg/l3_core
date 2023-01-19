//=================================================================================================
// Copyright (c) 2023, Alexander Stumpf, Felix Sternkopf TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef L3_LIBS_STDLIB_TYPES_H__
#define L3_LIBS_STDLIB_TYPES_H__

#include <mutex>
#include <shared_mutex>
#include <memory>

namespace l3
{
/*
 * Typedefs based on std library
 */
#if __cpp_lib_shared_mutex
typedef std::shared_mutex Mutex;
typedef std::shared_lock<Mutex> SharedLock;
#else
typedef std::mutex Mutex;
#endif

typedef std::unique_lock<Mutex> UniqueLock;
//typedef std::upgrade_lock<Mutex> UpgradeLock;
//typedef std::upgrade_to_unique_lock<Mutex> UpgradeToUniqueLock;



template<class T>
using SharedPtr = std::shared_ptr<T>;

template <class T, class... Args>
inline SharedPtr<T> makeShared(Args&&... args)
{
  return std::make_shared<T>(std::forward<Args>(args)...);
}

template<class T>
using UniquePtr = std::unique_ptr<T>;

#if __cpp_lib_make_unique
template <class T, class... Args>
inline UniquePtr<T> makeUnique(Args&&... args)
{
  return std::make_unique<T>(std::forward<Args>(args)...);
}
#else
template <class T, class... Args>
inline UniquePtr<T> makeUnique(Args&&... args)
{
  return UniquePtr<T>(new T(std::forward<Args>(args)...));
}
#endif

template<class T1, class T2>
inline SharedPtr<T1> staticPointerCast(const SharedPtr<T2>& p) noexcept
{
  return std::static_pointer_cast<T1>(p);
}

template<class T1, class T2>
inline SharedPtr<T1> dynamicPointerCast(const SharedPtr<T2>& p) noexcept
{
  return std::dynamic_pointer_cast<T1>(p);
}

template<class T1, class T2>
inline SharedPtr<T1> constPointerCast(const SharedPtr<T2>& p) noexcept
{
  return std::const_pointer_cast<T1>(p);
}
}  // namespace l3

#endif
