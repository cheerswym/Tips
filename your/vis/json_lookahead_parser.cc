
#include "onboard/vis/json_lookahead_parser.h"

namespace qcraft {

bool LookaheadParserHandler::Null() {
  st_ = kHasNull;
  v_.SetNull();
  return true;
}
bool LookaheadParserHandler::Bool(bool b) {
  st_ = kHasBool;
  v_.SetBool(b);
  return true;
}
bool LookaheadParserHandler::Int(int i) {
  st_ = kHasNumber;
  v_.SetInt(i);
  return true;
}
bool LookaheadParserHandler::Uint(unsigned u) {
  st_ = kHasNumber;
  v_.SetUint(u);
  return true;
}
bool LookaheadParserHandler::Int64(int64_t i) {
  st_ = kHasNumber;
  v_.SetInt64(i);
  return true;
}
bool LookaheadParserHandler::Uint64(uint64_t u) {
  st_ = kHasNumber;
  v_.SetUint64(u);
  return true;
}
bool LookaheadParserHandler::Double(double d) {
  st_ = kHasNumber;
  v_.SetDouble(d);
  return true;
}
bool LookaheadParserHandler::RawNumber(const char*, rapidjson::SizeType, bool) {
  return false;
}
bool LookaheadParserHandler::String(const char* str, rapidjson::SizeType length,
                                    bool) {
  st_ = kHasString;
  v_.SetString(str, length);
  return true;
}
bool LookaheadParserHandler::StartObject() {
  st_ = kEnteringObject;
  return true;
}
bool LookaheadParserHandler::Key(const char* str, rapidjson::SizeType length,
                                 bool) {
  st_ = kHasKey;
  v_.SetString(str, length);
  return true;
}
bool LookaheadParserHandler::EndObject(rapidjson::SizeType) {
  st_ = kExitingObject;
  return true;
}
bool LookaheadParserHandler::StartArray() {
  st_ = kEnteringArray;
  return true;
}
bool LookaheadParserHandler::EndArray(rapidjson::SizeType) {
  st_ = kExitingArray;
  return true;
}

LookaheadParserHandler::LookaheadParserHandler(char* str)
    : v_(), st_(kInit), r_(), ss_(str) {
  r_.IterativeParseInit();
  ParseNext();
}

void LookaheadParserHandler::ParseNext() {
  if (r_.HasParseError()) {
    st_ = kError;
    return;
  }

  r_.IterativeParseNext<parseFlags>(ss_, *this);
}

bool LookaheadParser::EnterObject() {
  if (st_ != kEnteringObject) {
    st_ = kError;
    return false;
  }

  ParseNext();
  return true;
}

bool LookaheadParser::EnterArray() {
  if (st_ != kEnteringArray) {
    st_ = kError;
    return false;
  }

  ParseNext();
  return true;
}

const char* LookaheadParser::NextObjectKey() {
  if (st_ == kHasKey) {
    const char* result = v_.GetString();
    ParseNext();
    return result;
  }

  if (st_ != kExitingObject) {
    st_ = kError;
    return 0;
  }

  ParseNext();
  return 0;
}

bool LookaheadParser::NextArrayValue() {
  if (st_ == kExitingArray) {
    ParseNext();
    return false;
  }

  if (st_ == kError || st_ == kExitingObject || st_ == kHasKey) {
    st_ = kError;
    return false;
  }

  return true;
}

int LookaheadParser::GetInt() {
  if (st_ != kHasNumber || !v_.IsInt()) {
    st_ = kError;
    return 0;
  }

  int result = v_.GetInt();
  ParseNext();
  return result;
}

double LookaheadParser::GetDouble() {
  if (st_ != kHasNumber) {
    st_ = kError;
    return 0.;
  }

  double result = v_.GetDouble();
  ParseNext();
  return result;
}

bool LookaheadParser::GetBool() {
  if (st_ != kHasBool) {
    st_ = kError;
    return false;
  }

  bool result = v_.GetBool();
  ParseNext();
  return result;
}

void LookaheadParser::GetNull() {
  if (st_ != kHasNull) {
    st_ = kError;
    return;
  }

  ParseNext();
}

const char* LookaheadParser::GetString() {
  if (st_ != kHasString) {
    st_ = kError;
    return 0;
  }

  const char* result = v_.GetString();
  ParseNext();
  return result;
}

void LookaheadParser::SkipOut(int depth) {
  do {
    if (st_ == kEnteringArray || st_ == kEnteringObject) {
      ++depth;
    } else if (st_ == kExitingArray || st_ == kExitingObject) {
      --depth;
    } else if (st_ == kError) {
      return;
    }

    ParseNext();
  } while (depth > 0);
}

void LookaheadParser::SkipValue() { SkipOut(0); }

void LookaheadParser::SkipArray() { SkipOut(1); }

void LookaheadParser::SkipObject() { SkipOut(1); }

rapidjson::Value* LookaheadParser::PeekValue() {
  if (st_ >= kHasNull && st_ <= kHasKey) {
    return &v_;
  }

  return 0;
}

int LookaheadParser::PeekType() {
  if (st_ >= kHasNull && st_ <= kHasKey) {
    return v_.GetType();
  }

  if (st_ == kEnteringArray) {
    return rapidjson::kArrayType;
  }

  if (st_ == kEnteringObject) {
    return rapidjson::kObjectType;
  }

  return -1;
}

}  // namespace qcraft
