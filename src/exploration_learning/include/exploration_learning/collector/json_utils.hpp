#pragma once

#include <string>

namespace exploration_learning::collector
{

/// @brief 将普通字符串转义为 JSON string 内容。
/// @param value 待转义字符串。
/// @return 可安全写入 JSON string 的转义结果，不包含外层引号。
std::string escape_json_string(const std::string & value);

/// @brief 判断字符串是否可以作为 JSON object 或 array 原样嵌入。
/// @param value 待判断字符串。
/// @return true 表示字符串首尾看起来是 JSON object 或 array。
bool looks_like_json_value(const std::string & value);

/// @brief 将原始 payload 包装成稳定 JSON 字段。
/// @param value 上游 topic 的原始 payload。
/// @return 若 payload 看起来是 JSON，则返回 {"raw_json":payload}，否则返回 {"raw_text":"..."}。
std::string wrap_raw_payload(const std::string & value);

}  // namespace exploration_learning::collector
