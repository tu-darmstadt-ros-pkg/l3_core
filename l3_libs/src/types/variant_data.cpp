#include <l3_libs/types/variant_data.h>

#include <l3_libs/types/typedefs.h>

namespace l3
{
VariantData::VariantData()
  : any()
  , type_hash_(0)
  , type_name_()
{}

VariantData::VariantData(const VariantData& in)
  : any(static_cast<const any&>(in))
  , type_hash_(in.type_hash_)
  , type_name_(in.type_name_)
{}

VariantData::VariantData(VariantData&& in)
  : any(std::move(static_cast<any&&>(in)))
  , type_hash_(std::move(in.type_hash_))
  , type_name_(std::move(in.type_name_))
{}

VariantData& VariantData::swap(VariantData& other)
{
  any::swap(static_cast<any&>(other));
  std::swap(type_hash_, other.type_hash_);
  std::swap(type_name_, other.type_name_);
  return *this;
}

VariantData& VariantData::operator=(const VariantData& other)
{
  VariantData(other).swap(*this);
  return *this;
}

VariantData& VariantData::operator=(VariantData&& other)
{
  other.swap(*this);
  // VariantData().swap(other);
  return *this;
}

void VariantData::clear()
{
  any::clear();
  type_hash_ = 0;
  type_name_.clear();
}

ByteStream& operator<<(ByteStream& stream, const VariantData& in)
{
  ROS_ASSERT(in.hasValue());
  const std::string& type = in.type();

  if (type.empty())
  {
    ROS_ERROR("VariantData: Empty type given, cannot serialize!");
    return stream;
  }

  // auto-dispatch type
  if (VariantDataSerialization::canSerialize(in))
  {
    const VariantDataSerialization::Serializer& serializer = VariantDataSerialization::getSerializer(in);
    stream << in.type_hash_;
    return serializer(stream, in);
  }

  ROS_ERROR("VariantData<%s> cannot be serialized! Type could not be dispatched.", type.c_str());
  return stream;
}

ByteStream& operator>>(ByteStream& stream, VariantData& out)
{
  size_t type_hash;
  stream >> type_hash;

  if (type_hash == 0)
  {
    ROS_ERROR("VariantData: Empty type given, cannot deserialize!");
    return stream;
  }

  // auto-dispatch type
  if (VariantDataSerialization::canSerialize(type_hash))
  {
    const VariantDataSerialization::Deserializer& deserializer = VariantDataSerialization::getDeserializer(type_hash);
    return deserializer(stream, out);
  }

  ROS_ERROR("VariantData cannot be deserialized! Unknown type.");
  return stream;
}

void variantDataSetMsgToL3(const l3_msgs::VariantDataSet& msg, VariantDataSet& data_set)
{
  ROS_ASSERT(msg.keys.size() == msg.values.size());

  for (size_t i = 0; i < msg.keys.size(); i++)
    variantDataMsgToL3(msg.values[i], data_set[msg.keys[i].data]);
}

void variantDataSetL3ToMsg(const VariantDataSet& data_set, l3_msgs::VariantDataSet& msg)
{
  msg.keys.clear();
  msg.keys.reserve(data_set.size());

  msg.values.clear();
  msg.values.reserve(data_set.size());

  for (const VariantDataPair& p : data_set)
  {
    std_msgs::String key_msg;
    key_msg.data = p.first;
    msg.keys.push_back(key_msg);

    l3_msgs::VariantData value_msg;
    variantDataL3ToMsg(p.second, value_msg);
    msg.values.push_back(value_msg);
  }
}

VariantDataSet::VariantDataSet()
  : std::unordered_map<std::string, VariantData>()
{}

VariantDataSerialization::VariantDataSerialization()
  : serializers_()
  , deserializers_()
{}
}  // namespace l3

L3_VARIANT_DATA_REGISTER_DEFAULT_SERIALIZATION(bool)
L3_VARIANT_DATA_REGISTER_DEFAULT_SERIALIZATION(short)
L3_VARIANT_DATA_REGISTER_DEFAULT_SERIALIZATION(int)
L3_VARIANT_DATA_REGISTER_DEFAULT_SERIALIZATION(unsigned int)
L3_VARIANT_DATA_REGISTER_DEFAULT_SERIALIZATION(long)
L3_VARIANT_DATA_REGISTER_DEFAULT_SERIALIZATION(long long)
L3_VARIANT_DATA_REGISTER_DEFAULT_SERIALIZATION(unsigned long)
L3_VARIANT_DATA_REGISTER_DEFAULT_SERIALIZATION(float)
L3_VARIANT_DATA_REGISTER_DEFAULT_SERIALIZATION(double)
L3_VARIANT_DATA_REGISTER_DEFAULT_SERIALIZATION(char)
L3_VARIANT_DATA_REGISTER_DEFAULT_SERIALIZATION(std::string)
L3_VARIANT_DATA_REGISTER_DEFAULT_SERIALIZATION(l3::Vector3)
L3_VARIANT_DATA_REGISTER_DEFAULT_SERIALIZATION(l3::Vector4)
L3_VARIANT_DATA_REGISTER_DEFAULT_SERIALIZATION(l3::Pose2D)
L3_VARIANT_DATA_REGISTER_DEFAULT_SERIALIZATION(l3::Pose)
L3_VARIANT_DATA_REGISTER_DEFAULT_SERIALIZATION(l3::Transform)
