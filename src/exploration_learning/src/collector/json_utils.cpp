#include "exploration_learning/collector/json_utils.hpp"

#include <cctype>
#include <iomanip>
#include <sstream>

namespace exploration_learning::collector
{

std::string escape_json_string(const std::string & value)
{
  std::ostringstream escaped;
  for (const char ch : value) {
    switch (ch) {
      case '\\':
        escaped << "\\\\";
        break;
      case '"':
        escaped << "\\\"";
        break;
      case '\b':
        escaped << "\\b";
        break;
      case '\f':
        escaped << "\\f";
        break;
      case '\n':
        escaped << "\\n";
        break;
      case '\r':
        escaped << "\\r";
        break;
      case '\t':
        escaped << "\\t";
        break;
      default:
        if (static_cast<unsigned char>(ch) < 0x20U) {
          escaped << "\\u"
                  << std::hex << std::setw(4) << std::setfill('0')
                  << static_cast<int>(static_cast<unsigned char>(ch))
                  << std::dec << std::setfill(' ');
        } else {
          escaped << ch;
        }
        break;
    }
  }
  return escaped.str();
}

bool looks_like_json_value(const std::string & value)
{
  auto first = value.begin();
  while (first != value.end() && std::isspace(static_cast<unsigned char>(*first))) {
    ++first;
  }
  if (first == value.end()) {
    return false;
  }

  auto last = value.end();
  do {
    --last;
  } while (last != first && std::isspace(static_cast<unsigned char>(*last)));

  return ((*first == '{' && *last == '}') || (*first == '[' && *last == ']'));
}

std::string wrap_raw_payload(const std::string & value)
{
  if (looks_like_json_value(value)) {
    return std::string("{\"raw_json\":") + value + "}";
  }
  return std::string("{\"raw_text\":\"") + escape_json_string(value) + "\"}";
}

}  // namespace exploration_learning::collector
