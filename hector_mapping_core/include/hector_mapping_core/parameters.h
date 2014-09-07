//=================================================================================================
// Copyright (c) 2013, Johannes Meyer and contributors, Technische Universitat Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
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


#ifndef HECTOR_MAPPING_PARAMETERS_H
#define HECTOR_MAPPING_PARAMETERS_H

#include <typeinfo>
#include <map>
#include <boost/shared_ptr.hpp>

namespace hector_mapping {

template <typename T> class Parameter;

class ParameterBase
{
public:
  virtual ~ParameterBase() {}
  virtual bool empty() { return true; }
  template <typename T> Parameter<T>& cast()             { return dynamic_cast<Parameter<T> &>(*this); }
  template <typename T> const Parameter<T>& cast() const { return dynamic_cast<const Parameter<T> &>(*this); }
};
typedef boost::shared_ptr<ParameterBase> ParameterPtr;
typedef boost::shared_ptr<const ParameterBase> ParameterConstPtr;

namespace internal {
  struct null_deleter { void operator()(void const *) const {} };
}

template <typename T> class Parameter : public ParameterBase
{
private:
  boost::shared_ptr<T> t_;
  T default_value_;

public:
  typedef T type;

  Parameter() : default_value_() {}
  Parameter(const Parameter<T>& other) : t_(other.t_), default_value_(other.default_value) {}
  Parameter(T& value) : t_(&value, internal::null_deleter()), default_value_() {}
  virtual ~Parameter() {}

  virtual bool empty() const { return !t_; }

  operator T& () { return get(); }
  operator const T& () const { return get(); }
  T& get() { if (empty()) *this = default_value(); return *t_; }
  const T& get() const { if (empty()) return default_value(); return *t_; }
  T* get_pointer() const { return t_.get(); }

  Parameter<T>& default_value(const T& default_value) {
    default_value_ = default_value;
    if (empty()) *this = default_value_;
    return *this;
  }
  const T& default_value() const { return default_value_; }

  Parameter<T>& operator=(const T& value) {
    if (!t_) t_.reset(new T());
    *t_ = value;
    return *this;
  }

  Parameter<T>& operator=(const Parameter<T>& other) {
    t_ = other.t_;
    return *this;
  }
};

class Parameters
{
private:
  typedef std::string Key;
  typedef std::map<Key,ParameterPtr> ParameterMap;
  boost::shared_ptr<ParameterMap> parameters_;

public:
  Parameters() : parameters_(new ParameterMap()) {}
  Parameters(const Parameters& other) : parameters_(other.parameters_) {}
  ~Parameters() {}

  template <typename T> Parameter<T>& operator()(const std::string& name, const T& default_value) const { return add<T>(name).default_value(default_value); }
  template <typename T> Parameter<T>& operator()(const std::string& name, T& value) const { return add<T>(name, value); }
  template <typename T> Parameter<T>& operator()(const std::string& name, Parameter<T>& parameter) const { return add<T>(name, parameter); }
  template <typename T> Parameter<T>& operator()(const std::string& name) const { return get<T>(name); }

  bool has(const std::string& name) const { return parameters_->count(name); }

  // template <typename T> Parameter<T>& get(const std::string& name) {
  template <typename T> T& get(const std::string& name) {
    if (!has(name)) return add<T>(name);
    return parameters_->at(name)->cast<T>();
  }

  // template <typename T> const Parameter<T>& get(const std::string& name) const {
  template <typename T> const T& get(const std::string& name) const {
    if (!has(name)) add<T>(name);
    return parameters_->at(name)->cast<T>();
  }

  template <typename T> Parameter<T>& add(const std::string& name) const {
    if (has(name)) return parameters_->at(name)->cast<T>();
    Parameter<T> *parameter = new Parameter<T>();
    parameters_->insert(std::make_pair<Key,ParameterPtr>(name, ParameterPtr(parameter)));
    return *parameter;
  }

  template <typename T> Parameter<T>& add(const std::string& name, T& value) const {
    if (has(name)) {
      value = parameters_->at(name)->cast<T>();
      parameters_->erase(name);
    }
    Parameter<T> *parameter = new Parameter<T>(value);
    parameters_->insert(std::make_pair<Key,ParameterPtr>(name, ParameterPtr(parameter)));
    return *parameter;
  }

  template <typename T> Parameter<T>& add(const std::string& name, Parameter<T>& parameter) const {
    if (has(name)) {
      parameter = parameters_->at(name)->cast<T>();
      parameters_->erase(name);
    }
    parameters_->insert(std::make_pair<Key,ParameterPtr>(name, ParameterPtr(&parameter, internal::null_deleter())));
    return parameter;
  }

  template <typename T> void remove(const std::string& name)  const {
    ParameterMap::iterator it = parameters_->find(name);
    if (it != parameters_->end()) parameters_->erase(it);
  }

private:
  ParameterPtr byName(const std::string& name) const {
    static ParameterPtr empty(new ParameterBase());
    if (!parameters_->count(name)) return empty;
    return parameters_->at(name);
  }
};

} // namespace hector_mapping

#endif // HECTOR_MAPPING_PARAMETERS_H
