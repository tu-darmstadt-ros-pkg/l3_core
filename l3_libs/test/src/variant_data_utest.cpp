#include <ros/ros.h>

#include <gtest/gtest.h>

#include <l3_libs/test_macros.h>
#include <l3_libs/types/types.h>

using namespace l3;

// Test for checking functionality of VariantData
TEST(VariantData, Basics)
{
  // basic test
  VariantData integer(int(42));
  EXPECT_NO_THROW(integer.value<int>());
  EXPECT_ANY_THROW(integer.value<double>());
  EXPECT_TRUE(integer.hasValue());
  EXPECT_EQ(std::string("int"), integer.type());
  EXPECT_EQ(42, integer.value<int>());

  // check operator=
  integer = VariantData(int(1));
  EXPECT_EQ(1, integer.value<int>());

  integer = 123;
  EXPECT_EQ(123, integer.value<int>());

  // check if correct template overload is used (due to danger of wrong move operator usage)
  VariantData integer2;
  integer2 = integer;
  EXPECT_EQ(123, integer.value<int>());
  EXPECT_EQ(123, integer2.value<int>());

  // misc tests
  *integer.pointer<int>() = 42;
  EXPECT_EQ(42, integer.value<int>());

  integer.clear();
  EXPECT_FALSE(integer.hasValue());
}

TEST(VariantData, IntegralSerialization)
{
  /// test basic datatypes
  VariantData bool_type(true);
  VariantData short_type((short)7);
  VariantData int_type(42);
  VariantData uint_type(21u);
  VariantData long_type(5l);
  VariantData long_long_type(10ll);
  VariantData float_type(3.14f);
  VariantData double_type(3.14);
  VariantData char_type('A');
  VariantData string_type(std::string("Hello"));

  /// test serialization
  ByteStream s;
  s << bool_type;
  s << short_type;
  s << int_type;
  s << uint_type;
  s << long_type;
  s << long_long_type;
  s << float_type;
  s << double_type;
  s << char_type;
  s << string_type;

  bool_type.clear();
  short_type.clear();
  int_type.clear();
  uint_type.clear();
  long_type.clear();
  long_long_type.clear();
  float_type.clear();
  double_type.clear();
  char_type.clear();
  string_type.clear();

  EXPECT_FALSE(bool_type.hasValue());
  EXPECT_FALSE(short_type.hasValue());
  EXPECT_FALSE(int_type.hasValue());
  EXPECT_FALSE(uint_type.hasValue());
  EXPECT_FALSE(long_type.hasValue());
  EXPECT_FALSE(long_long_type.hasValue());
  EXPECT_FALSE(float_type.hasValue());
  EXPECT_FALSE(double_type.hasValue());
  EXPECT_FALSE(char_type.hasValue());
  EXPECT_FALSE(string_type.hasValue());

  // deserialize
  s >> bool_type;
  s >> short_type;
  s >> int_type;
  s >> uint_type;
  s >> long_type;
  s >> long_long_type;
  s >> float_type;
  s >> double_type;
  s >> char_type;
  s >> string_type;

  EXPECT_EQ(0, s.remainingUnreadBytes());

  EXPECT_TRUE(bool_type.value<bool>());
  EXPECT_EQ((short)7, short_type.value<short>());
  EXPECT_EQ(42, int_type.value<int>());
  EXPECT_EQ(21u, uint_type.value<unsigned int>());
  EXPECT_EQ(5l, long_type.value<long>());
  EXPECT_EQ(10l, long_long_type.value<long long>());
  EXPECT_FLOAT_EQ(3.14f, float_type.value<float>());
  EXPECT_DOUBLE_EQ(3.14, double_type.value<double>());
  EXPECT_EQ('A', char_type.value<char>());
  EXPECT_EQ(std::string("Hello"), string_type.value<std::string>());

  // test vector
  std::vector<VariantData> vector;
  vector.push_back(int_type);
  vector.push_back(float_type);
  vector.push_back(char_type);

  s << vector;
  vector.clear();
  s >> vector;

  EXPECT_EQ(0, s.remainingUnreadBytes());

  EXPECT_EQ(3, vector.size());
  EXPECT_EQ(42, vector[0].value<int>());
  EXPECT_EQ(3.14f, vector[1].value<float>());
  EXPECT_EQ('A', vector[2].value<char>());

  // test map
  VariantDataSet set;
  set["bool"] = bool_type;
  set["double"] = double_type;
  set["string"] = string_type;

  s << set;
  set.clear();
  s >> set;

  EXPECT_EQ(0, s.remainingUnreadBytes());

  EXPECT_EQ(3, set.size());
  EXPECT_TRUE(set["bool"].value<bool>());
  EXPECT_DOUBLE_EQ(3.14, set["double"].value<double>());
  EXPECT_EQ(std::string("Hello"), set["string"].value<std::string>());

  // check if original VariantDatas are still valid (cross out wrong move operator calls)
  EXPECT_TRUE(bool_type.value<bool>());
  EXPECT_EQ((short)7, short_type.value<short>());
  EXPECT_EQ(42, int_type.value<int>());
  EXPECT_EQ(21u, uint_type.value<unsigned int>());
  EXPECT_EQ(5l, long_type.value<long>());
  EXPECT_EQ(10l, long_long_type.value<long long>());
  EXPECT_FLOAT_EQ(3.14f, float_type.value<float>());
  EXPECT_DOUBLE_EQ(3.14, double_type.value<double>());
  EXPECT_EQ('A', char_type.value<char>());
  EXPECT_EQ(std::string("Hello"), string_type.value<std::string>());
}

TEST(VariantData, VariantDataMsg)
{
  VariantDataSet set;
  set["bool"] = VariantData(true);
  set["int"] = VariantData(42);
  set["uint"] = VariantData(21u);
  set["double"] = VariantData(3.14);
  set["char"] = VariantData('A');
  set["string"] = VariantData(std::string("Hello"));

  l3_msgs::VariantDataSet set_msg;
  variantDataSetL3ToMsg(set, set_msg);
  set.clear();
  variantDataSetMsgToL3(set_msg, set);

  EXPECT_EQ(6, set.size());
  EXPECT_TRUE(set["bool"].value<bool>());
  EXPECT_EQ(42, set["int"].value<int>());
  EXPECT_EQ(21u, set["uint"].value<unsigned int>());
  EXPECT_DOUBLE_EQ(3.14, set["double"].value<double>());
  EXPECT_EQ('A', set["char"].value<char>());
  EXPECT_EQ(std::string("Hello"), set["string"].value<std::string>());
}

TEST(VariantData, L3_TypesSerialization)
{
  std_msgs::Header header;
  header.frame_id = "test";
  header.stamp = ros::Time(42.0);

  VariantData Vector3_type(Vector3(1.0, 2.0, 3.0));
  VariantData Vector4_type(Vector4(1.0, 2.0, 3.0, 4.0));
  VariantData Pose2D_type(Pose2D(1.0, 2.0, M_PI));
  VariantData Pose_type(Pose(1.0, 2.0, 3.0, 1.1, 1.2, 1.3));
  VariantData Transform_type(Transform(1.0, 2.0, 3.0, 1.1, 1.2, 1.3));
  VariantData FloatingBase_type(FloatingBase(7, Pose_type.value<Pose>(), header));
  VariantData Foothold_type(Foothold(2, Pose_type.value<Pose>(), header));

  /// test serialization
  ByteStream s;
  s << Vector3_type;
  s << Vector4_type;
  s << Pose2D_type;
  s << Pose_type;
  s << Transform_type;
  s << FloatingBase_type;
  s << Foothold_type;

  Vector3_type.clear();
  Vector4_type.clear();
  Pose2D_type.clear();
  Pose_type.clear();
  Transform_type.clear();
  FloatingBase_type.clear();
  Foothold_type.clear();

  EXPECT_FALSE(Vector3_type.hasValue());
  EXPECT_FALSE(Vector4_type.hasValue());
  EXPECT_FALSE(Pose2D_type.hasValue());
  EXPECT_FALSE(Pose_type.hasValue());
  EXPECT_FALSE(Transform_type.hasValue());
  EXPECT_FALSE(FloatingBase_type.hasValue());
  EXPECT_FALSE(Foothold_type.hasValue());

  s >> Vector3_type;
  s >> Vector4_type;
  s >> Pose2D_type;
  s >> Pose_type;
  s >> Transform_type;
  s >> FloatingBase_type;
  s >> Foothold_type;

  EXPECT_EQ(0, s.remainingUnreadBytes());

  EXPECT_VECTOR_EQ(Vector3(1.0, 2.0, 3.0), Vector3_type.value<Vector3>());

  EXPECT_DOUBLE_EQ(1.0, Vector4_type.value<Vector4>().x());
  EXPECT_DOUBLE_EQ(2.0, Vector4_type.value<Vector4>().y());
  EXPECT_DOUBLE_EQ(3.0, Vector4_type.value<Vector4>().z());
  EXPECT_DOUBLE_EQ(4.0, Vector4_type.value<Vector4>().w());

  EXPECT_DOUBLE_EQ(1.0, Pose2D_type.value<Pose2D>().x());
  EXPECT_DOUBLE_EQ(2.0, Pose2D_type.value<Pose2D>().y());
  EXPECT_DOUBLE_EQ(M_PI, Pose2D_type.value<Pose2D>().yaw());

  EXPECT_POSE_EQ(Pose(1.0, 2.0, 3.0, 1.1, 1.2, 1.3), Pose_type.value<Pose>());

  EXPECT_POSE_EQ(Transform(1.0, 2.0, 3.0, 1.1, 1.2, 1.3), Transform_type.value<Transform>());

  EXPECT_EQ(7, FloatingBase_type.value<FloatingBase>().idx);
  EXPECT_POSE_EQ(Pose(1.0, 2.0, 3.0, 1.1, 1.2, 1.3), FloatingBase_type.value<FloatingBase>().pose());
  EXPECT_EQ(std::string("test"), FloatingBase_type.value<FloatingBase>().header.frame_id);
  EXPECT_DOUBLE_EQ(42.0, FloatingBase_type.value<FloatingBase>().header.stamp.toSec());

  EXPECT_EQ(2, Foothold_type.value<Foothold>().idx);
  EXPECT_POSE_EQ(Pose(1.0, 2.0, 3.0, 1.1, 1.2, 1.3), Foothold_type.value<Foothold>().pose());
  EXPECT_EQ(std::string("test"), Foothold_type.value<Foothold>().header.frame_id);
  EXPECT_DOUBLE_EQ(42.0, Foothold_type.value<Foothold>().header.stamp.toSec());
}

TEST(VariantData, SharedPtrSerialization)
{
  VariantData std_type = std::make_shared<std::string>("Hello std");
  VariantData boost_type = boost::make_shared<std::string>("Hello boost");

  /// test serialization
  ByteStream s;
  s << std_type;
  s << boost_type;

  std_type.clear();
  boost_type.clear();

  EXPECT_FALSE(std_type.hasValue());
  EXPECT_FALSE(boost_type.hasValue());

  s >> std_type;
  s >> boost_type;

  EXPECT_EQ(0, s.remainingUnreadBytes());

  // check types
  EXPECT_NO_THROW(std_type.value<std::shared_ptr<std::string>>());
  EXPECT_NO_THROW(boost_type.value<boost::shared_ptr<std::string>>());

  // check ptr
  std::shared_ptr<std::string> std_string = std_type.value<std::shared_ptr<std::string>>();
  EXPECT_FALSE(std_string.get() == nullptr);
  EXPECT_EQ(std::string("Hello std"), *std_string);

  boost::shared_ptr<std::string> boost_string = boost_type.value<boost::shared_ptr<std::string>>();
  EXPECT_FALSE(boost_string.get() == nullptr);
  EXPECT_EQ(std::string("Hello boost"), *boost_string);
}

TEST(VariantData, ContainerSerialization)
{
  VariantData list_type = std::list<int>({ 0, 1, 1, 2, 3, 5 });
  VariantData vector_type = std::vector<std::string>({ "Hello", "World", "!" });

  /// test serialization
  ByteStream s;
  s << list_type;
  s << vector_type;

  list_type.clear();
  vector_type.clear();

  EXPECT_FALSE(list_type.hasValue());
  EXPECT_FALSE(vector_type.hasValue());

  s >> list_type;
  s >> vector_type;

  EXPECT_EQ(0, s.remainingUnreadBytes());

  // check types
  EXPECT_NO_THROW(list_type.value<std::list<int>>());
  EXPECT_ANY_THROW(list_type.value<std::vector<int>>());
  EXPECT_ANY_THROW(list_type.value<int>());

  EXPECT_NO_THROW(vector_type.value<std::vector<std::string>>());
  EXPECT_ANY_THROW(vector_type.value<std::list<std::string>>());
  EXPECT_ANY_THROW(vector_type.value<std::string>());

  // check list
  const std::list<int>& list = list_type.value<std::list<int>>();
  EXPECT_EQ(6, list.size());
  std::list<int>::const_iterator itr = list.begin();
  EXPECT_EQ(0, *(itr++));
  EXPECT_EQ(1, *(itr++));
  EXPECT_EQ(1, *(itr++));
  EXPECT_EQ(2, *(itr++));
  EXPECT_EQ(3, *(itr++));
  EXPECT_EQ(5, *(itr++));

  // check vector
  const std::vector<std::string>& vector = vector_type.value<std::vector<std::string>>();
  EXPECT_EQ(3, vector.size());
  EXPECT_EQ(std::string("Hello"), vector[0]);
  EXPECT_EQ(std::string("World"), vector[1]);
  EXPECT_EQ(std::string("!"), vector[2]);
}
