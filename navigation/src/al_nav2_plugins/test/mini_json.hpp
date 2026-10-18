// Copyright 2026 Matvey Razgulov
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

// Minimal JSON reader for the gtest fixtures (no external dependency, so the
// tests build in any ROS 2 Humble image). Supports objects, arrays, numbers,
// strings, true/false/null; \uXXXX escapes are kept verbatim.
#pragma once

#include <cstdlib>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace mini_json
{

struct Value
{
  enum Type {Null, Bool, Num, Str, Arr, Obj} type{Null};
  double num{0.0};
  bool boolean{false};
  std::string str;
  std::vector<Value> arr;
  std::vector<std::pair<std::string, Value>> obj;

  const Value & operator[](const std::string & key) const
  {
    for (const auto & kv : obj) {
      if (kv.first == key) {
        return kv.second;
      }
    }
    throw std::runtime_error("json: missing key '" + key + "'");
  }
  const Value & operator[](std::size_t i) const {return arr.at(i);}
  bool has(const std::string & key) const
  {
    for (const auto & kv : obj) {
      if (kv.first == key) {
        return true;
      }
    }
    return false;
  }
  std::size_t size() const {return type == Arr ? arr.size() : obj.size();}
  double d() const
  {
    if (type != Num) {
      throw std::runtime_error("json: not a number");
    }
    return num;
  }
  int i() const {return static_cast<int>(d());}
  const std::string & s() const {return str;}
};

class Parser
{
public:
  explicit Parser(const std::string & text)
  : t_(text) {}
  Value parse()
  {
    Value v = value();
    ws();
    if (p_ != t_.size()) {
      throw std::runtime_error("json: trailing characters");
    }
    return v;
  }

private:
  void ws() {while (p_ < t_.size() && std::isspace(static_cast<unsigned char>(t_[p_]))) {++p_;}}
  char peek() {ws(); if (p_ >= t_.size()) {throw std::runtime_error("json: eof");} return t_[p_];}
  void expect(char c)
  {
    if (peek() != c) {
      throw std::runtime_error(std::string("json: expected '") + c + "'");
    }
    ++p_;
  }
  bool lit(const char * w)
  {
    const std::string s(w);
    if (t_.compare(p_, s.size(), s) == 0) {
      p_ += s.size();
      return true;
    }
    return false;
  }
  std::string string_()
  {
    expect('"');
    std::string out;
    while (p_ < t_.size() && t_[p_] != '"') {
      if (t_[p_] == '\\' && p_ + 1 < t_.size()) {
        const char e = t_[p_ + 1];
        switch (e) {
          case 'n': out += '\n'; break;
          case 't': out += '\t'; break;
          case 'r': out += '\r'; break;
          case 'b': out += '\b'; break;
          case 'f': out += '\f'; break;
          case 'u': out += "\\u"; break;
          default: out += e;
        }
        p_ += 2;
      } else {
        out += t_[p_++];
      }
    }
    expect('"');
    return out;
  }
  Value value()
  {
    const char c = peek();
    Value v;
    if (c == '{') {
      ++p_;
      v.type = Value::Obj;
      if (peek() == '}') {++p_; return v;}
      for (;; ) {
        std::string k = string_();
        expect(':');
        v.obj.emplace_back(std::move(k), value());
        if (peek() == ',') {++p_; continue;}
        expect('}');
        return v;
      }
    }
    if (c == '[') {
      ++p_;
      v.type = Value::Arr;
      if (peek() == ']') {++p_; return v;}
      for (;; ) {
        v.arr.push_back(value());
        if (peek() == ',') {++p_; continue;}
        expect(']');
        return v;
      }
    }
    if (c == '"') {v.type = Value::Str; v.str = string_(); return v;}
    if (lit("true")) {v.type = Value::Bool; v.boolean = true; return v;}
    if (lit("false")) {v.type = Value::Bool; return v;}
    if (lit("null")) {return v;}
    char * end = nullptr;
    v.num = std::strtod(t_.c_str() + p_, &end);
    if (end == t_.c_str() + p_) {
      throw std::runtime_error("json: bad token at " + std::to_string(p_));
    }
    p_ = static_cast<std::size_t>(end - t_.c_str());
    v.type = Value::Num;
    return v;
  }

  const std::string & t_;
  std::size_t p_{0};
};

inline Value parseFile(const std::string & path)
{
  std::ifstream f(path);
  if (!f) {
    throw std::runtime_error("cannot open " + path);
  }
  std::stringstream ss;
  ss << f.rdbuf();
  const std::string text = ss.str();
  return Parser(text).parse();
}

}  // namespace mini_json
