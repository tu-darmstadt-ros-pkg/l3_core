//=================================================================================================
// Copyright (c) 2023, Alexander Stumpf, TU Darmstadt
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

#ifndef L3_VARIANT_DATA_MACROS_H__
#define L3_VARIANT_DATA_MACROS_H__

#include <list>
#include <vector>
#include <memory>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <l3_libs/conversions/serialization.h>

/*
 * Macros defining the creation of default serializers
 */

#define L3_VARIANT_DATA_DEFAULT_SERIALIZER(ValueType) [](l3::ByteStream& stream, const l3::VariantData& in) -> l3::ByteStream& { return stream << in.value<ValueType>(); }

#define L3_VARIANT_DATA_DEFAULT_DESERIALIZER(ValueType)                                                                                                                            \
  [](l3::ByteStream& stream, l3::VariantData& out) -> l3::ByteStream& {                                                                                                            \
    ValueType val;                                                                                                                                                                 \
    stream >> val;                                                                                                                                                                 \
    out = std::move(val);                                                                                                                                                          \
    return stream;                                                                                                                                                                 \
  }

#define L3_VARIANT_DATA_DEFAULT_DESERIALIZER_NON_MOVEABLE(ValueType)                                                                                                               \
  [](l3::ByteStream& stream, l3::VariantData& out) -> l3::ByteStream& {                                                                                                            \
    ValueType val;                                                                                                                                                                 \
    stream >> val;                                                                                                                                                                 \
    out = val;                                                                                                                                                                     \
    return stream;                                                                                                                                                                 \
  }

#define L3_VARIANT_DATA_STD_SHARED_PTR_SERIALIZER(ValueType)                                                                                                                       \
  [](l3::ByteStream& stream, const l3::VariantData& in) -> l3::ByteStream& {                                                                                                       \
    std::shared_ptr<ValueType> ptr = in.value<std::shared_ptr<ValueType>>();                                                                                                       \
    stream << *ptr;                                                                                                                                                                \
    return stream;                                                                                                                                                                 \
  }

#define L3_VARIANT_DATA_STD_SHARED_PTR_DESERIALIZER(ValueType)                                                                                                                     \
  [](l3::ByteStream& stream, l3::VariantData& out) -> l3::ByteStream& {                                                                                                            \
    std::shared_ptr<ValueType> ptr = std::make_shared<ValueType>();                                                                                                                \
    stream >> *ptr;                                                                                                                                                                \
    out = ptr;                                                                                                                                                                     \
    return stream;                                                                                                                                                                 \
  }

#define L3_VARIANT_DATA_BOOST_SHARED_PTR_SERIALIZER(ValueType)                                                                                                                     \
  [](l3::ByteStream& stream, const l3::VariantData& in) -> l3::ByteStream& {                                                                                                       \
    boost::shared_ptr<ValueType> ptr = in.value<boost::shared_ptr<ValueType>>();                                                                                                   \
    stream << *ptr;                                                                                                                                                                \
    return stream;                                                                                                                                                                 \
  }

#define L3_VARIANT_DATA_BOOST_SHARED_PTR_DESERIALIZER(ValueType)                                                                                                                   \
  [](l3::ByteStream& stream, l3::VariantData& out) -> l3::ByteStream& {                                                                                                            \
    boost::shared_ptr<ValueType> ptr = boost::make_shared<ValueType>();                                                                                                            \
    stream >> *ptr;                                                                                                                                                                \
    out = ptr;                                                                                                                                                                     \
    return stream;                                                                                                                                                                 \
  }

#define L3_VARIANT_DATA_CONTAINER_SERIALIZER(ContainerType, ValueType)                                                                                                             \
  [](l3::ByteStream& stream, const l3::VariantData& in) -> l3::ByteStream& {                                                                                                       \
    const ContainerType<ValueType>& container = in.value<ContainerType<ValueType>>();                                                                                              \
    stream << container.size();                                                                                                                                                    \
    for (const ValueType& val : container)                                                                                                                                         \
      stream << val;                                                                                                                                                               \
    return stream;                                                                                                                                                                 \
  }

#define L3_VARIANT_DATA_CONTAINER_DESERIALIZER(ContainerType, ValueType)                                                                                                           \
  [](l3::ByteStream& stream, l3::VariantData& out) -> l3::ByteStream& {                                                                                                            \
    ContainerType<ValueType> container;                                                                                                                                            \
    size_t num;                                                                                                                                                                    \
    stream >> num;                                                                                                                                                                 \
    for (size_t i = 0; i < num; i++)                                                                                                                                               \
    {                                                                                                                                                                              \
      ValueType val;                                                                                                                                                               \
      stream >> val;                                                                                                                                                               \
      container.push_back(val);                                                                                                                                                    \
    }                                                                                                                                                                              \
    out = std::move(container);                                                                                                                                                    \
    return stream;                                                                                                                                                                 \
  }

/*
 * @macro Internal macro for registration of serializers
 */

#define L3_VARIANT_DATA_REGISTER_SERIALIZER_INTERNAL(ValueType, Serializer, UniqueID)                                                                                              \
  namespace                                                                                                                                                                        \
  {                                                                                                                                                                                \
  using namespace l3;                                                                                                                                                              \
  struct ProxyExec##UniqueID                                                                                                                                                       \
  {                                                                                                                                                                                \
    ProxyExec##UniqueID()                                                                                                                                                          \
    {                                                                                                                                                                              \
      VariantDataSerialization::registerSerializer<ValueType>(Serializer);                                                                                                         \
      VariantDataSerialization::registerSerializer<std::shared_ptr<ValueType>>(L3_VARIANT_DATA_STD_SHARED_PTR_SERIALIZER(ValueType));                                              \
      VariantDataSerialization::registerSerializer<boost::shared_ptr<ValueType>>(L3_VARIANT_DATA_BOOST_SHARED_PTR_SERIALIZER(ValueType));                                          \
      VariantDataSerialization::registerSerializer<std::list<ValueType>>(L3_VARIANT_DATA_CONTAINER_SERIALIZER(std::list, ValueType));                                              \
      VariantDataSerialization::registerSerializer<std::vector<ValueType>>(L3_VARIANT_DATA_CONTAINER_SERIALIZER(std::vector, ValueType));                                          \
      ROS_DEBUG("Registered Serializer<%s>", getTypeName<ValueType>().c_str());                                                                                                    \
    }                                                                                                                                                                              \
  };                                                                                                                                                                               \
  static ProxyExec##UniqueID g_register_serializer_##UniqueID;                                                                                                                     \
  }  // namespace l3

/**
 * @macro Internal macro required for preprocessor step to resolve __COUNTER__ argument.
 */
#define L3_VARIANT_DATA_REGISTER_SERIALIZER_HOP(ValueType, Serializer, UniqueID) L3_VARIANT_DATA_REGISTER_SERIALIZER_INTERNAL(ValueType, Serializer, UniqueID)

/**
 * @macro This macro registers a serializer as std::function<ByteStream&(ByteStream&, const VariantData&)> for the given type.
 * This also includes std container (std::list and std::vector) specialized for this type.
 * It must be used in the source (*.cpp) file!
 */
#define L3_VARIANT_DATA_REGISTER_SERIALIZER(ValueType, Serializer) L3_VARIANT_DATA_REGISTER_SERIALIZER_HOP(ValueType, Serializer, __COUNTER__)

/**
 * @macro This macro registers a default serializer for the given type using a standard template method.
 * This also includes std container (std::list and std::vector) specialized for this type.
 * This does only work, if the type is memory-aligned and must be used in the source (*.cpp) file!
 */
#define L3_VARIANT_DATA_REGISTER_DEFAULT_SERIALIZER(ValueType) L3_VARIANT_DATA_REGISTER_SERIALIZER(ValueType, L3_VARIANT_DATA_DEFAULT_SERIALIZER(ValueType))

/*
 * @macro Internal macro for registration of deserializers
 */

#define L3_VARIANT_DATA_REGISTER_DESERIALIZER_INTERNAL(ValueType, Deserializer, UniqueID)                                                                                          \
  namespace                                                                                                                                                                        \
  {                                                                                                                                                                                \
  using namespace l3;                                                                                                                                                              \
  struct ProxyExec##UniqueID                                                                                                                                                       \
  {                                                                                                                                                                                \
    ProxyExec##UniqueID()                                                                                                                                                          \
    {                                                                                                                                                                              \
      VariantDataSerialization::registerDeserializer<ValueType>(Deserializer);                                                                                                     \
      VariantDataSerialization::registerDeserializer<std::shared_ptr<ValueType>>(L3_VARIANT_DATA_STD_SHARED_PTR_DESERIALIZER(ValueType));                                          \
      VariantDataSerialization::registerDeserializer<boost::shared_ptr<ValueType>>(L3_VARIANT_DATA_BOOST_SHARED_PTR_DESERIALIZER(ValueType));                                      \
      VariantDataSerialization::registerDeserializer<std::list<ValueType>>(L3_VARIANT_DATA_CONTAINER_DESERIALIZER(std::list, ValueType));                                          \
      VariantDataSerialization::registerDeserializer<std::vector<ValueType>>(L3_VARIANT_DATA_CONTAINER_DESERIALIZER(std::vector, ValueType));                                      \
      ROS_DEBUG("Registered Deserializer<%s>", getTypeName<ValueType>().c_str());                                                                                                  \
    }                                                                                                                                                                              \
  };                                                                                                                                                                               \
  static ProxyExec##UniqueID g_register_deserializer_##UniqueID;                                                                                                                   \
  }  // namespace l3

/**
 * @macro Internal macro required for preprocessor step to resolve __COUNTER__ argument.
 */
#define L3_VARIANT_DATA_REGISTER_DESERIALIZER_HOP(ValueType, Deserializer, UniqueID) L3_VARIANT_DATA_REGISTER_DESERIALIZER_INTERNAL(ValueType, Deserializer, UniqueID)

/**
 * @macro This macro registers a deserializer as std::function<ByteStream&(ByteStream&, VariantData&)> for the given type.
 * This also includes std container (std::list and std::vector) specialized for this type.
 * It must be used in the source (*.cpp) file!
 */
#define L3_VARIANT_DATA_REGISTER_DESERIALIZER(ValueType, Deserializer) L3_VARIANT_DATA_REGISTER_DESERIALIZER_HOP(ValueType, Deserializer, __COUNTER__)

/**
 * @macro This macro registers a default deserializer for the given type using a standard template method.
 * This also includes std container (std::list and std::vector) specialized for this type.
 * This does only work, if the type is memory-aligned and must be used in the source (*.cpp) file!
 */
#define L3_VARIANT_DATA_REGISTER_DEFAULT_DESERIALIZER(ValueType) L3_VARIANT_DATA_REGISTER_DESERIALIZER(ValueType, L3_VARIANT_DATA_DEFAULT_DESERIALIZER(ValueType))

/*
 * Combo macros
 */

/**
 * @macro This macro registers a serializer and a deserializer for the given type;
 * Serializer given as std::function<ByteStream&(ByteStream&, const VariantData&)>
 * Deserializer given as std::function<ByteStream&(ByteStream&, VariantData&)>
 * This also includes std container (std::list and std::vector) specialized for this type.
 */
#define L3_VARIANT_DATA_REGISTER_SERIALIZATION(ValueType, Serializer, Deserializer)                                                                                                \
  L3_VARIANT_DATA_REGISTER_SERIALIZER(ValueType, Serializer)                                                                                                                       \
  L3_VARIANT_DATA_REGISTER_DESERIALIZER(ValueType, Deserializer)

/**
 * @macro This macro registers a default serializer as std::function<ByteStream&(ByteStream&, const VariantData&)>
 * and a deserializer as std::function<ByteStream&(ByteStream&, VariantData&)> for the given type.
 * This also includes std container (std::list and std::vector) specialized for this type.
 * This does only work, if the type is memory-aligned and must be used in the source (*.cpp) file!
 */
#define L3_VARIANT_DATA_REGISTER_DEFAULT_SERIALIZATION(ValueType)                                                                                                                  \
  L3_VARIANT_DATA_REGISTER_DEFAULT_SERIALIZER(ValueType)                                                                                                                           \
  L3_VARIANT_DATA_REGISTER_DEFAULT_DESERIALIZER(ValueType)

#endif
