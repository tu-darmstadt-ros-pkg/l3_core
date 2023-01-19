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

#ifndef L3_LIBS_BOOST_TYPES_H__
#define L3_LIBS_BOOST_TYPES_H__

#include <boost/thread/mutex.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/move/unique_ptr.hpp>
#include <boost/move/make_unique.hpp>

namespace l3
{
/*
 * Typedefs based on boost library
 */
template <class T>
using SharedPtr = boost::shared_ptr<T>;

template <class T, class... Args>
inline SharedPtr<T> makeShared(Args&&... args)
{
  return boost::make_shared<T>(boost::forward<Args>(args)...);
}

template <class T>
using UniquePtr = boost::movelib::unique_ptr<T>;

template <class T, class... Args>
inline UniquePtr<T> makeUnique(Args&&... args)
{
  return boost::movelib::make_unique<T>(boost::forward<Args>(args)...);
}

template <class T1, class T2>
inline SharedPtr<T1> staticPointerCast(const SharedPtr<T2>& p) noexcept
{
  return boost::static_pointer_cast<T1>(p);
}

template <class T1, class T2>
inline SharedPtr<T1> dynamicPointerCast(const SharedPtr<T2>& p) noexcept
{
  return boost::dynamic_pointer_cast<T1>(p);
}

template <class T1, class T2>
inline SharedPtr<T1> constPointerCast(const SharedPtr<T2>& p) noexcept
{
  return boost::const_pointer_cast<T1>(p);
}

// SFINAE helper
template <class T>
struct is_shared_ptr : std::false_type
{
};

template <class T>
struct is_shared_ptr<SharedPtr<T>> : std::true_type
{
};

using Mutex = boost::shared_mutex;

using SharedLock = boost::shared_lock<Mutex>;
using SharedLockPtr = SharedPtr<SharedLock>;

using UniqueLock = boost::unique_lock<Mutex>;
using UniqueLockPtr = SharedPtr<UniqueLock>;

using UpgradeLock = boost::upgrade_lock<Mutex>;
using UpgradeLockPtr = SharedPtr<UpgradeLock>;

using UpgradeToUniqueLock = boost::upgrade_to_unique_lock<Mutex>;
using UpgradeToUniqueLockPtr = SharedPtr<UpgradeToUniqueLock>;
}  // namespace l3

#endif
