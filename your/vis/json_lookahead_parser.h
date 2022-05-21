#ifndef ONBOARD_VIS_JSON_LOOKAHEAD_PARSER_H_
#define ONBOARD_VIS_JSON_LOOKAHEAD_PARSER_H_

#include "rapidjson/document.h"
#include "rapidjson/reader.h"

namespace qcraft {

class LookaheadParserHandler {
 public:
  bool Null();
  bool Bool(bool b);
  bool Int(int i);
  bool Uint(unsigned u);
  bool Int64(int64_t i);
  bool Uint64(uint64_t u);
  bool Double(double d);
  bool RawNumber(const char*, rapidjson::SizeType, bool);
  bool String(const char* str, rapidjson::SizeType length, bool);
  bool StartObject();
  bool Key(const char* str, rapidjson::SizeType length, bool);
  bool EndObject(rapidjson::SizeType);
  bool StartArray();
  bool EndArray(rapidjson::SizeType);

 protected:
  explicit LookaheadParserHandler(char* str);
  void ParseNext();

 protected:
  enum LookaheadParsingState {
    kInit,
    kError,
    kHasNull,
    kHasBool,
    kHasNumber,
    kHasString,
    kHasKey,
    kEnteringObject,
    kExitingObject,
    kEnteringArray,
    kExitingArray
  };

  rapidjson::Value v_;
  LookaheadParsingState st_;
  rapidjson::Reader r_;
  rapidjson::InsituStringStream ss_;

  static const int parseFlags =
      rapidjson::kParseDefaultFlags | rapidjson::kParseInsituFlag;
};

class LookaheadParser : protected LookaheadParserHandler {
 public:
  explicit LookaheadParser(char* str) : LookaheadParserHandler(str) {}

  bool EnterObject();
  bool EnterArray();
  const char* NextObjectKey();
  bool NextArrayValue();
  int GetInt();
  double GetDouble();
  const char* GetString();
  bool GetBool();
  void GetNull();

  void SkipObject();
  void SkipArray();
  void SkipValue();
  rapidjson::Value* PeekValue();
  int PeekType();  // returns a rapidjson::Type, or -1 for no value (at end of
                   // object/array)

  bool IsValid() { return st_ != kError; }

 protected:
  void SkipOut(int depth);
};

}  // namespace qcraft

#endif
