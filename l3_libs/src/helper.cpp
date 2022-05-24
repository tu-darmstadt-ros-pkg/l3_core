#include <l3_libs/helper.h>

namespace l3
{
std::string toString(const XmlRpc::XmlRpcValue::Type& type)
{
  switch (type)
  {
    case XmlRpc::XmlRpcValue::TypeInvalid:
      return "Invalid";
    case XmlRpc::XmlRpcValue::TypeBoolean:
      return "Bool";
    case XmlRpc::XmlRpcValue::TypeInt:
      return "Int";
    case XmlRpc::XmlRpcValue::TypeDouble:
      return "Double";
    case XmlRpc::XmlRpcValue::TypeString:
      return "String";
    case XmlRpc::XmlRpcValue::TypeDateTime:
      return "DateTime";
    case XmlRpc::XmlRpcValue::TypeBase64:
      return "Base64";
    case XmlRpc::XmlRpcValue::TypeArray:
      return "Array";
    case XmlRpc::XmlRpcValue::TypeStruct:
      return "Struct";
  }
  return "Unknown";
}
}  // namespace l3
