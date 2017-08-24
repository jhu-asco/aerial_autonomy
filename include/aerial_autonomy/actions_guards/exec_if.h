#pragma once
// Copyright Aleksey Gurtovoy 2000-2008
//
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//
// See http://www.boost.org/libs/mpl for documentation.

// $Id: exec_if.hpp 55648 2009-08-18 05:16:53Z agurtovoy $
// $Date: 2009-08-17 22:16:53 -0700 (Mon, 17 Aug 2009) $
// $Revision: 55648 $

/**
 * Modified find_if that executes actions
 * until one of the actions returns true.
 * It stops executing after that.
 *
 * Modified by Gowtham Garimella.
 * email ggarime1[at]jhu.edu
 */

#include <boost/mpl/apply.hpp>
#include <boost/mpl/assert.hpp>
#include <boost/mpl/aux_/unwrap.hpp>
#include <boost/mpl/begin_end.hpp>
#include <boost/mpl/bool.hpp>
#include <boost/mpl/deref.hpp>
#include <boost/mpl/identity.hpp>
#include <boost/mpl/is_sequence.hpp>
#include <boost/mpl/next_prior.hpp>

#include <boost/type_traits/is_same.hpp>
#include <boost/utility/value_init.hpp>

/**
 * @brief namespace for boost library
 */
namespace boost {
/**
 * @brief namespace for mpl module
 */
namespace mpl {

/**
 * @brief namespace for aux module
 */
namespace aux {

/**
 * @brief Default implementation of exec which returns true
 *
 * @tparam done If done is false, there is a specialized implementation
 */
template <bool done = true> struct exec_if_impl {
  template <typename Iterator, typename LastIterator, typename TransformFunc,
            typename F>
  /**
   * @brief Default implementation that returns true
   *
   * @param  Initial position in the vector
   * @param  Final position in the vector
   * @param  Function used to transform the elements in vector before calling
   * @param F Functor to call on the elements in the vector
   *
   * @return true if Functor evaluation returned True/False otherwise
   */
  static bool execute(Iterator *, LastIterator *, TransformFunc *, F) {
    return true;
  }
};

/**
 * @brief Specialized implementation for done  = false
 */
template <> struct exec_if_impl<false> {
  /**
   * @brief
   *
   * @tparam Iterator  Vector iterator
   * @tparam LastIterator  Vector iterator
   * @tparam TransformFunc  Transform function to be applied on the vector
   * elements
   * @tparam F Functor to call on the elements of the vector
   * @param  Initial position in the vector
   * @param  Final position in the vector
   * @param  Function used to transform the elements in vector before calling
   * functor on them
   * @param f Functor to call on the elements of the vector
   *
   * @return False if the functor call returned false. Otherwise will evaluate
   * the next element in the vector
   */
  template <typename Iterator, typename LastIterator, typename TransformFunc,
            typename F>
  static bool execute(Iterator *, LastIterator *, TransformFunc *, F f) {
    typedef typename deref<Iterator>::type item;
    typedef typename apply1<TransformFunc, item>::type arg;

    // dwa 2002/9/10 -- make sure not to invoke undefined behavior
    // when we pass arg.
    value_initialized<arg> x;
    // Only execute further actions if return from the current action is true
    if (aux::unwrap(f, 0)(boost::get(x))) {
      typedef typename mpl::next<Iterator>::type iter;
      return exec_if_impl<boost::is_same<iter, LastIterator>::value>::execute(
          static_cast<iter *>(0), static_cast<LastIterator *>(0),
          static_cast<TransformFunc *>(0), f);
    }
    return false;
  }
};

} // namespace aux

// agurt, 17/mar/02: pointer default parameters are necessary to workaround
// MSVC 6.5 function template signature's mangling bug
/**
 * @brief Function to go through elements of boost sequence and apply the
 * functor F on each element. If the functor F returns false in between
 * the function stops executing further elements and returns false
 *
 * @tparam Sequence Sequence type to iterate on
 * @tparam TransformOp Operation to transform the elements in the sequence
 * @tparam F Functor type to apply on the element sequence
 * @param f functor instance
 * @param  Sequence pointer (Sequence elements are instantiated by default)
 * @param  Transform operation pointer not used (Instantiated internally)
 *
 * @return  If the functor F returns false in between
 * the function stops executing further elements and returns false. Otherwise
 * returns true.
 */
template <typename Sequence, typename TransformOp, typename F>
inline bool exec_if(F f, Sequence * = 0, TransformOp * = 0) {
  BOOST_MPL_ASSERT((is_sequence<Sequence>));

  typedef typename begin<Sequence>::type first;
  typedef typename end<Sequence>::type last;

  return aux::exec_if_impl<boost::is_same<first, last>::value>::execute(
      static_cast<first *>(0), static_cast<last *>(0),
      static_cast<TransformOp *>(0), f);
}

/**
 * @brief Simplified implementation without transform operation
 *
 * @tparam Sequence Sequence type to iterate on
 * @tparam F Functor type to apply on the element sequence
 * @param f functor instance
 * @param  Sequence pointer (Sequence elements are instantiated by default)
 *
 * @return  If the functor F returns false in between
 * the function stops executing further elements and returns false. Otherwise
 * returns true.
 */
template <typename Sequence, typename F>
inline bool exec_if(F f, Sequence * = 0) {
  return exec_if<Sequence, identity<>>(f);
}
}
}
