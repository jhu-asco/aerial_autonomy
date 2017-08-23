#pragma once

// Copyright (c) 2012, Sebastian Jeltsch (sjeltsch@kip.uni-heidelberg.de)
// Distributed under the terms of the GPLv2 or newer

#include <map>
#include <memory>
#include <string>
#include <type_traits>

#include "fab/bind.h"
#include "fab/exception.h"

#define FAB_ASSERT_RETURN_TYPE(TYPE)                                           \
  static_assert(                                                               \
      std::is_pointer<TYPE>::value &&                                          \
          std::is_base_of<Base,                                                \
                          typename std::remove_pointer<TYPE>::type>::value,    \
      "Callable needs to return Base*");

namespace fab {

template <typename Base> class BaseFunction {
public:
  typedef Base *return_type;
  virtual ~BaseFunction() {}
};

template <typename Base, typename... Args>
class Function : public BaseFunction<Base> {
public:
  typedef std::function<typename BaseFunction<Base>::return_type(Args...)>
      function_type;

  template <typename T> Function(T &&func) : _func(std::forward<T>(func)) {}

  typename function_type::result_type operator()(Args &&... args) const {
    return _func(std::forward<Args>(args)...);
  }

protected:
  function_type _func;
};

template <typename Base, typename Key = std::string> struct Factory {
public:
  typedef std::unique_ptr<Base> return_type;

  template <typename... Args>
  return_type Create(Key const &key, Args &&... args);

  /// registers lvalue std::functions
  template <typename Return, typename... Args>
  void Register(Key const &key,
                std::function<Return(Args... args)> const &delegate);

  /// registers rvalue std::functions
  template <typename Return, typename... Args>
  void Register(Key const &key, std::function<Return(Args... args)> &&delegate);

  /// registers function pointer
  template <typename Return, typename... Args>
  void Register(Key const &key, Return (*delegate)(Args... args));

  /// registers zero-argument lambdas like []() { return ...; }
  template <typename Lambda>
  void Register(Key const &key, Lambda const &lambda);

protected:
  typedef Key key_type;
  typedef std::unique_ptr<BaseFunction<Base>> value_type;
  std::map<key_type, value_type> _map;
};

// implementation

template <typename Base, typename Key>
template <typename... Args>
typename Factory<Base, Key>::return_type
Factory<Base, Key>::Create(Key const &key, Args &&... args) {
  auto ret = _map.find(key);
  if (ret == _map.end())
    throw exception::UnknownKey();

  typedef Function<Base, Args...> wrapper_t;
  try {
    wrapper_t const &wrapper = dynamic_cast<wrapper_t const &>(*(ret->second));
    return return_type(wrapper(std::forward<Args>(args)...));
  } catch (std::bad_cast &e) {
    throw exception::BadArguments();
  }
}

template <typename Base, typename Key>
template <typename Return, typename... Args>
void Factory<Base, Key>::Register(
    Key const &key, std::function<Return(Args... args)> const &delegate) {
  FAB_ASSERT_RETURN_TYPE(Return)
  _map[key] = value_type(new Function<Base, Args...>(delegate));
}

template <typename Base, typename Key>
template <typename Return, typename... Args>
void Factory<Base, Key>::Register(
    Key const &key, std::function<Return(Args... args)> &&delegate) {
  FAB_ASSERT_RETURN_TYPE(Return)
  _map[key] = value_type(new Function<Base, Args...>(std::move(delegate)));
}

template <typename Base, typename Key>
template <typename Return, typename... Args>
void Factory<Base, Key>::Register(Key const &key,
                                  Return (*delegate)(Args... args)) {
  FAB_ASSERT_RETURN_TYPE(Return)
  _map[key] = value_type(new Function<Base, Args...>(delegate));
}

template <typename Base, typename Key>
template <typename Lambda>
void Factory<Base, Key>::Register(Key const &key, Lambda const &lambda) {
  auto f = fab::bind(&Lambda::operator(), lambda);
  FAB_ASSERT_RETURN_TYPE(typename decltype(f)::result_type)
  Register(key, std::move(f));
}

} // fab
