//=================================================================================================
// Copyright (c) 2022, Alexander Stumpf, TU Darmstadt
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

#ifndef L3_VARIANT_DATA_H__
#define L3_VARIANT_DATA_H__

#ifdef __cpp_lib_any
#include <any>
#else
#include <boost/any.hpp>
#endif

#include <typeindex>
#include <unordered_map>

#include <l3_msgs/VariantDataSet.h>

#include <l3_libs/singleton.h>
#include <l3_libs/conversions/serialization.h>
#include <l3_libs/types/type_traits.h>
#include <l3_libs/types/variant_data_macros.h>

namespace l3
{
#ifdef __cpp_lib_any
typedef std::any any;
#else
typedef boost::any any;
#endif

/**
 * @brief Wrapper for any in order to enable I/O (serialization) of any types.
 */
class VariantData : private any
{
public:
  /** Constructors */

  VariantData();

  template <class ValueType>
  VariantData(const ValueType& in)
    : any(in)
    , type_hash_(l3::getTypeHash<ValueType>())
    , type_name_(l3::getTypeName<ValueType>())
  {}

  template <class ValueType>
  VariantData(ValueType&& in)
    : any(std::move(in))
    , type_hash_(l3::getTypeHash<ValueType>())
    , type_name_(l3::getTypeName<ValueType>())
  {}

  VariantData(const VariantData& in);
  inline VariantData(VariantData& in)  // required as template specialization
    : VariantData(static_cast<const VariantData&>(in))
  {}
  VariantData(VariantData&& in);

  /** Copy Operators */

  VariantData& swap(VariantData& other);

  VariantData& operator=(const VariantData& other);
  inline VariantData& operator=(VariantData& other) { return operator=(static_cast<const VariantData&>(other)); }  // required as template specialization
  VariantData& operator=(VariantData&& other);

  template <class ValueType>
  VariantData& operator=(const ValueType& other)
  {
    // apply copy-swap idiom
    VariantData(ValueType(other)).swap(*this);
    return *this;
  }

  template <class ValueType>
  VariantData& operator=(ValueType& other)
  {
    // apply copy-swap idiom
    VariantData(ValueType(other)).swap(*this);
    return *this;
  }

  // perfect forwarding of ValueType
  template <class ValueType>
  VariantData& operator=(ValueType&& other)
  {
    VariantData(std::move(other)).swap(*this);
    return *this;
  }

  /** Serialization Operators */

  template <class ValueType>
  friend inline bool operator<<(VariantData& data, const ValueType& in)
  {
    data = VariantData(in);
    return true;
  }

  template <class ValueType>
  friend inline bool operator>>(const VariantData& data, ValueType& out)
  {
    return data.value(out);
  }

  friend ByteStream& operator<<(ByteStream& stream, const VariantData& in);
  friend ByteStream& operator>>(ByteStream& stream, VariantData& out);

  /** Data Operators */

  inline const std::string& type() const { return type_name_; }

  void clear();

  template <class ValueType>
  constexpr bool isType() const
  {
    return l3::getTypeHash<ValueType>() == type_hash_;
  }

  template <class ValueType>
  constexpr bool isType(ValueType) const
  {
    return isType<ValueType>();
  }

  inline size_t getTypeHash() const { return type_hash_; }

#ifdef __cpp_lib_any
  inline bool hasValue() const { return this->has_value(); }

  template <class ValueType>
  inline ValueType* pointer()
  {
    return isType<ValueType>() ? :: ::any_cast<ValueType>(this) : nullptr;
  }

  template <class ValueType>
  inline const ValueType& value()
  {
    return std::any_cast<const ValueType&>(*this);
  }

  template <class ValueType>
  inline ValueType& value()
  {
    return std::any_cast<ValueType&>(*this);
  }

  template <class ValueType>
  inline bool value(ValueType& val)
  {
    if (isType<ValueType>())
    {
      val = std::any_cast<ValueType&>(*this);
      return true;
    }
    else
      return false;
  }
#else
  inline bool hasValue() const { return !empty(); }

  template <class ValueType>
  inline ValueType* pointer()
  {
    return isType<ValueType>() ? boost::any_cast<ValueType>(this) : nullptr;
  }

  template <class ValueType>
  inline const ValueType& value() const
  {
    return boost::any_cast<const ValueType&>(*this);
  }

  template <class ValueType>
  inline ValueType& value()
  {
    return boost::any_cast<ValueType&>(*this);
  }

  template <class ValueType>
  inline bool value(ValueType& val) const
  {
    if (isType<ValueType>())
    {
      val = boost::any_cast<const ValueType&>(*this);
      return true;
    }
    else
      return false;
  }
#endif

private:
  size_t type_hash_;
  std::string type_name_;
};

typedef std::pair<const std::string, VariantData> VariantDataPair;

/**
 * @brief The VariantDataSet represents a simple lookup table which addresses
 * single VariantData using a string identifier
 */
class VariantDataSet : public std::unordered_map<std::string, VariantData>
{
public:
  VariantDataSet();

  inline const VariantData& operator[](const std::string& name) const { return find(name)->second; }
  inline VariantData& operator[](const std::string& name) { return std::unordered_map<std::string, VariantData>::operator[](name); }

  friend inline ByteStream& operator<<(ByteStream& stream, const VariantDataSet& in)
  {
    return operator<<(stream, static_cast<const std::unordered_map<std::string, VariantData>&>(in));
  }
  friend inline ByteStream& operator>>(ByteStream& stream, VariantDataSet& out)
  {
    return operator>>(stream, static_cast<std::unordered_map<std::string, VariantData>&>(out));
  }

  /**
   * @brief Checks if any entity with given name exists
   * @param name Name to lookup
   * @return True, if an entity was found
   */
  bool has(const std::string& name) const
  {
    VariantDataSet::const_iterator itr = find(name);
    return itr != end();
  }

  /**
   * @brief Checks if any entity of given type and name exists
   * @param ValueType Desired type of the entity
   * @param name Name to lookup
   * @return True, if an entity with specified type was found
   */
  template <class ValueType>
  bool hasValue(const std::string& name) const
  {
    VariantDataSet::const_iterator itr = find(name);
    if (itr == end())
      return false;

    return itr->second.isType<ValueType>();
  }

  /**
   * @brief Type-safe getter of a specific entity. If either the name or type mismatches,
   * the given default value is returned.
   * @param ValueType Desired type of the entity
   * @param name Name to lookup
   * @param default_value Default value to be returned in case of a miss
   * @return The value of the found entity, otherwise the default value
   */
  template <class ValueType>
  ValueType getValue(const std::string& name, const ValueType& default_value) const
  {
    VariantDataSet::const_iterator itr = find(name);
    if (itr != end() && itr->second.isType<ValueType>())
      return itr->second.value<ValueType>();
    else
      return default_value;
  }

  /**
   * @brief Non-safe getter of a specific entity.
   * @param ValueType Desired type of the entity
   * @param name Name to lookup
   * @param val [out] Output variable to store the found data
   * @return True, if an entity with specified type was found
   */
  template <class ValueType>
  bool get(const std::string& name, ValueType& val) const
  {
    VariantDataSet::const_iterator itr = find(name);
    if (itr == end())
      return false;
    else
      return itr->second.value(val);
  }
};

/**
 * Serializers that are globally used to transform any VariantData and VariantDataSet into
 * a bytestream and vice versa.
 */
class VariantDataSerialization : public Singleton<VariantDataSerialization>
{
public:
  VariantDataSerialization();

  // typedefs
  typedef std::function<ByteStream&(ByteStream&, const VariantData&)> Serializer;
  typedef std::function<ByteStream&(ByteStream&, VariantData&)> Deserializer;

  /**
   * Registers template generated default serialization code for the given type.
   * WARNING: Does only work, if the type is memory-aligned.
   * @param ValueType type for which the serializer is written for
   */
  template <class ValueType>
  static void registerDefaultSerializer()
  {
    registerSerializer<ValueType>(L3_VARIANT_DATA_DEFAULT_SERIALIZER(ValueType));
  }

  /**
   * Registers template generated default deserialization code for the given type.
   * WARNING: Does only work, if the type is memory-aligned.
   * @param ValueType type for which the deserializer is written for
   */
  template <class ValueType, typename std::enable_if<std::is_move_constructible<ValueType>::value>::type>
  static void registerDefaultDeserializer()
  {
    registerDeserializer<ValueType>(L3_VARIANT_DATA_DEFAULT_DESERIALIZER(ValueType));
  }

  /**
   * Registers template generated default deserialization code for the given type.
   * WARNING: Does only work, if the type is memory-aligned.
   * @param ValueType type for which the deserializer is written for
   */
  template <class ValueType, typename std::enable_if<!std::is_move_constructible<ValueType>::value>::type>
  static void registerDefaultDeserializer()
  {
    registerDeserializer<ValueType>(L3_VARIANT_DATA_DEFAULT_DESERIALIZER_NON_MOVEABLE(ValueType));
  }

  /**
   * Registers user-written serialization code for the given type. The serialization code
   * must only consider the nested type as the VariantData members will be handled automatically
   * @param ValueType type for which the serializer is written for
   * @param s Serializer given as std::function<ByteStream&(ByteStream&, const VariantData&)>
   */
  template <class ValueType>
  static void registerSerializer(const Serializer& s)
  {
    mutableInstance().serializers_[getTypeHash<ValueType>()] = s;
  }

  /**
   * Registers user-written deserialization code for the given type. The deserialization code
   * must only consider the nested type as the VariantData members will be handled automatically.
   * @param ValueType type for which the deserializer is written for
   * @param s Deserializer given as std::function<ByteStream&(ByteStream&, VariantData&)>
   */
  template <class ValueType>
  static void registerDeserializer(const Deserializer& d)
  {
    mutableInstance().deserializers_[getTypeHash<ValueType>()] = d;
  }

  inline static bool canSerialize(const VariantData& data) { return canSerialize(data.getTypeHash()); }

  inline static bool canSerialize(const size_t& type_hash) { return instance().serializers_.find(type_hash) != instance().serializers_.end() &&
                                                                    instance().deserializers_.find(type_hash) != instance().deserializers_.end(); }

  template <class ValueType>
  static constexpr bool canSerialize()
  {
    return instance().serializers_.find(getTypeHash<ValueType>()) != instance().serializers_.end() &&
           instance().deserializers_.find(getTypeHash<ValueType>()) != instance().deserializers_.end();
  }

  inline static const Serializer& getSerializer(const VariantData& data) { return getSerializer(data.getTypeHash()); }

  inline static const Serializer& getSerializer(const size_t& type_hash) { return instance().serializers_.find(type_hash)->second; }

  template <class ValueType>
  inline static const Serializer& getSerializer()  { return instance().serializers_.find(getTypeHash<ValueType>())->second; }

  inline static const Deserializer& getDeserializer(const VariantData& data) { return getDeserializer(data.getTypeHash()); }

  inline static const Deserializer& getDeserializer(const size_t& type_hash) { return instance().deserializers_.find(type_hash)->second; }

  template <class ValueType>
  inline static const Deserializer& getDeserializer()  { return instance().deserializers_.find(getTypeHash<ValueType>())->second; }

private:
  // lookup table (cook book) that holds actually function pointers which can serialize a specific type
  typedef std::unordered_map<size_t, Serializer> SerializerMap;
  SerializerMap serializers_;

  // lookup table (cook book) that holds actually function pointers which can deserialize a specific type
  typedef std::unordered_map<size_t, Deserializer> DeserializerMap;
  DeserializerMap deserializers_;
};

/**
 * Helper functions
 */

inline void variantDataMsgToL3(const l3_msgs::VariantData& msg, VariantData& data) { vigir_generic_params::operator<<(data, msg.data); }
inline void variantDataL3ToMsg(const VariantData& data, l3_msgs::VariantData& msg) { vigir_generic_params::operator<<(msg.data, data); }

void variantDataSetMsgToL3(const l3_msgs::VariantDataSet& msg, VariantDataSet& data_set);
void variantDataSetL3ToMsg(const VariantDataSet& data_set, l3_msgs::VariantDataSet& msg);
}  // namespace l3

#endif
