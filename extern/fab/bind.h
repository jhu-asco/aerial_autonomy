#pragma once

// Copyright (c) 2012, Sebastian Jeltsch (sjeltsch@kip.uni-heidelberg.de)
// Distributed under the terms of the GPLv2 or newer

#include <functional>
#include <memory>
#include <type_traits>

namespace fab {

template <typename... Args> struct pack {};

typedef pack<decltype(std::placeholders::_1), decltype(std::placeholders::_2),
             decltype(std::placeholders::_3), decltype(std::placeholders::_4),
             decltype(std::placeholders::_5), decltype(std::placeholders::_6),
             decltype(std::placeholders::_7), decltype(std::placeholders::_8),
             decltype(std::placeholders::_9), decltype(std::placeholders::_10),
             decltype(std::placeholders::_11), decltype(std::placeholders::_12),
             decltype(std::placeholders::_13), decltype(std::placeholders::_14),
             decltype(std::placeholders::_15), decltype(std::placeholders::_16),
             decltype(std::placeholders::_17), decltype(std::placeholders::_18),
             decltype(std::placeholders::_19)>
    placeholders;

// copied from ZTL
template <size_t N, typename NoPack> struct get;

template <size_t N, template <typename...> class Pack, typename Arg0,
          typename... Args>
struct get<N, Pack<Arg0, Args...>> {
  typedef typename get<N - 1, Pack<Args...>>::type type;
};

template <template <typename...> class Pack, typename Arg0, typename... Args>
struct get<0, Pack<Arg0, Args...>> {
  typedef Arg0 type;
};

// adapted from ZTL
template <size_t To, typename NoPack = pack<>, size_t Idx = 0> struct range;

template <size_t To, size_t Idx, template <typename...> class Pack,
          typename... Args>
struct range<To, Pack<Args...>, Idx> {
  typedef Pack<Args..., std::integral_constant<size_t, Idx>> type;
};

template <size_t To, template <typename...> class Pack, typename... Args>
struct range<To, Pack<Args...>, To> {
  typedef Pack<Args...> type;
};

template <typename Ret, typename Lambda, template <typename...> class Pack,
          typename... Args, typename... Range>
std::function<Ret(Args...)> bind_helper(Ret (Lambda::*f)(Args...) const,
                                        Lambda const &l, Pack<Range...>) {
  return std::bind(f, l, typename get<Range::value, placeholders>::type()...);
}

template <typename Ret, typename Lambda, typename... Args>
std::function<Ret(Args...)> bind(Ret (Lambda::*f)(Args...) const,
                                 Lambda const &l) {
  return bind_helper(f, l, typename range<sizeof...(Args)>::type());
}

} // fab
