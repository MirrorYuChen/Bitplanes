/*
 * @Author: chenjingyu
 * @Date: 2023-11-24 16:12:02
 * @LastEditTime: 2023-11-24 16:13:30
 * @Description: Details
 * @FilePath: \CodeBlocks\C++\3rdLibs\circularbuffer\details.h
 */
#pragma once

#include <cassert>
#include <exception>
#include <iterator>
#include <memory>
#include <type_traits>
#include <utility>

// Silence MS /W4 warnings like C4913:
// "user defined binary operator ',' exists but no overload could convert all
// operands, default built-in binary operator ',' used" This might happen when
// previously including some boost headers that overload the coma operator.
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4913)
#endif

namespace cb {
namespace cb_details {

template <class Traits> struct nonconst_traits;

template <class ForwardIterator, class Diff, class T, class Alloc>
void uninitialized_fill_n_with_alloc(ForwardIterator first, Diff n,
                                     const T &item, Alloc &alloc);

template <class InputIterator, class ForwardIterator, class Alloc>
ForwardIterator uninitialized_copy(InputIterator first, InputIterator last,
                                   ForwardIterator dest, Alloc &a);

template <class InputIterator, class ForwardIterator, class Alloc>
ForwardIterator uninitialized_move_if_noexcept(InputIterator first,
                                               InputIterator last,
                                               ForwardIterator dest, Alloc &a);

/*!
    \struct const_traits
    \brief Defines the data types for a const iterator.
*/
template <class Traits> struct const_traits {
  // Basic types
  typedef typename Traits::value_type value_type;
  typedef typename Traits::const_pointer pointer;
  typedef const typename Traits::value_type &reference;
  typedef typename Traits::size_type size_type;
  typedef typename Traits::difference_type difference_type;

  // Non-const traits
  typedef nonconst_traits<Traits> nonconst_self;
};

/*!
    \struct nonconst_traits
    \brief Defines the data types for a non-const iterator.
*/
template <class Traits> struct nonconst_traits {
  // Basic types
  typedef typename Traits::value_type value_type;
  typedef typename Traits::pointer pointer;
  typedef typename Traits::value_type &reference;
  typedef typename Traits::size_type size_type;
  typedef typename Traits::difference_type difference_type;

  // Non-const traits
  typedef nonconst_traits<Traits> nonconst_self;
};

/*!
    \struct iterator_wrapper
    \brief Helper iterator dereference wrapper.
*/
template <class Iterator> struct iterator_wrapper {
  mutable Iterator m_it;
  explicit iterator_wrapper(Iterator it) : m_it(it) {}
  Iterator operator()() const { return m_it++; }

private:
  iterator_wrapper<Iterator> &
  operator=(const iterator_wrapper<Iterator> &); // do not generate
};

/*!
    \struct item_wrapper
    \brief Helper item dereference wrapper.
*/
template <class Pointer, class Value> struct item_wrapper {
  Value m_item;
  explicit item_wrapper(Value item) : m_item(item) {}
  Pointer operator()() const { return &m_item; }

private:
  item_wrapper<Pointer, Value> &
  operator=(const item_wrapper<Pointer, Value> &); // do not generate
};

/*!
    \struct assign_n
    \brief Helper functor for assigning n items.
*/
template <class Value, class Alloc> struct assign_n {
  typedef typename std::allocator_traits<Alloc>::size_type size_type;
  size_type m_n;
  Value m_item;
  Alloc &m_alloc;
  assign_n(size_type n, Value item, Alloc &alloc)
      : m_n(n), m_item(item), m_alloc(alloc) {}
  template <class Pointer> void operator()(Pointer p) const {
    uninitialized_fill_n_with_alloc(p, m_n, m_item, m_alloc);
  }

private:
  assign_n<Value, Alloc> &
  operator=(const assign_n<Value, Alloc> &); // do not generate
};

/*!
    \struct assign_range
    \brief Helper functor for assigning range of items.
*/
template <class Iterator, class Alloc> struct assign_range {
  Iterator m_first;
  Iterator m_last;
  Alloc &m_alloc;

  assign_range(const Iterator &first, const Iterator &last, Alloc &alloc)
      : m_first(first), m_last(last), m_alloc(alloc) {}

  template <class Pointer> void operator()(Pointer p) const {
    uninitialized_copy(m_first, m_last, p, m_alloc);
  }
};

template <class Iterator, class Alloc>
inline assign_range<Iterator, Alloc>
make_assign_range(const Iterator &first, const Iterator &last, Alloc &a) {
  return assign_range<Iterator, Alloc>(first, last, a);
}

/*!
    \class capacity_control
    \brief Capacity controller of the space optimized circular buffer.
*/
template <class Size> class capacity_control {

  //! The capacity of the space-optimized circular buffer.
  Size m_capacity;

  //! The lowest guaranteed or minimum capacity of the adapted space-optimized
  //! circular buffer.
  Size m_min_capacity;

public:
  //! Constructor.
  capacity_control(Size buffer_capacity, Size min_buffer_capacity = 0)
      : m_capacity(buffer_capacity),
        m_min_capacity(min_buffer_capacity) { // Check for capacity lower than
                                              // min_capacity.
    assert(buffer_capacity >= min_buffer_capacity);
  }

  // Default copy constructor.

  // Default assign operator.

  //! Get the capacity of the space optimized circular buffer.
  Size capacity() const { return m_capacity; }

  //! Get the minimal capacity of the space optimized circular buffer.
  Size min_capacity() const { return m_min_capacity; }

  //! Size operator - returns the capacity of the space optimized circular
  //! buffer.
  operator Size() const { return m_capacity; }
};

/*!
    \struct iterator
    \brief Random access iterator for the circular buffer.
    \param Buff The type of the underlying circular buffer.
    \param Traits Basic iterator types.
    \note This iterator is not circular. It was designed
          for iterating from begin() to end() of the circular buffer.
*/
template <class Buff, class Traits>
struct iterator
    : public std::iterator<
          std::random_access_iterator_tag, typename Traits::value_type,
          typename Traits::difference_type, typename Traits::pointer,
          typename Traits::reference> {
  // Helper types

  //! Base iterator.
  typedef std::iterator<std::random_access_iterator_tag,
                        typename Traits::value_type,
                        typename Traits::difference_type,
                        typename Traits::pointer, typename Traits::reference>
      base_iterator;

  //! Non-const iterator.
  typedef iterator<Buff, typename Traits::nonconst_self> nonconst_self;

  // Basic types

  //! The type of the elements stored in the circular buffer.
  typedef typename base_iterator::value_type value_type;

  //! Pointer to the element.
  typedef typename base_iterator::pointer pointer;

  //! Reference to the element.
  typedef typename base_iterator::reference reference;

  //! Size type.
  typedef typename Traits::size_type size_type;

  //! Difference type.
  typedef typename base_iterator::difference_type difference_type;

  // Member variables

  //! The circular buffer where the iterator points to.
  const Buff *m_buff;

  //! An internal iterator.
  pointer m_it;

  // Construction & assignment

  // Default copy constructor.

  //! Default constructor.
  iterator() : m_buff(0), m_it(0) {}

  iterator(const nonconst_self &it) : m_buff(it.m_buff), m_it(it.m_it) {}

  iterator(const Buff *cb, const pointer p) : m_buff(cb), m_it(p) {}

  //! Assign operator.
  iterator &operator=(const iterator &it) {
    if (this == &it)
      return *this;
    m_buff = it.m_buff;
    m_it = it.m_it;
    return *this;
  }

  // Random access iterator methods

  //! Dereferencing operator.
  reference operator*() const {
    assert(m_it != 0); // check for iterator pointing to end()
    return *m_it;
  }

  //! Dereferencing operator.
  pointer operator->() const { return &(operator*()); }

  //! Difference operator.
  template <class Traits0>
  difference_type operator-(const iterator<Buff, Traits0> &it) const {
    return linearize_pointer(*this) - linearize_pointer(it);
  }

  //! Increment operator (prefix).
  iterator &operator++() {
    assert(m_it != 0); // check for iterator pointing to end()
    m_buff->increment(m_it);
    if (m_it == m_buff->m_last)
      m_it = 0;
    return *this;
  }

  //! Increment operator (postfix).
  iterator operator++(int) {
    iterator<Buff, Traits> tmp = *this;
    ++*this;
    return tmp;
  }

  //! Decrement operator (prefix).
  iterator &operator--() {
    assert(m_it != m_buff->m_first); // check for iterator pointing to begin()
    if (m_it == 0)
      m_it = m_buff->m_last;
    m_buff->decrement(m_it);
    return *this;
  }

  //! Decrement operator (postfix).
  iterator operator--(int) {
    iterator<Buff, Traits> tmp = *this;
    --*this;
    return tmp;
  }

  //! Iterator addition.
  iterator &operator+=(difference_type n) {
    if (n > 0) {
      assert(m_buff->end() - *this >= n); // check for too large n
      m_it = m_buff->add(m_it, n);
      if (m_it == m_buff->m_last)
        m_it = 0;
    } else if (n < 0) {
      *this -= -n;
    }
    return *this;
  }

  //! Iterator addition.
  iterator operator+(difference_type n) const {
    return iterator<Buff, Traits>(*this) += n;
  }

  //! Iterator subtraction.
  iterator &operator-=(difference_type n) {
    if (n > 0) {
      assert(*this - m_buff->begin() >= n); // check for too large n
      m_it = m_buff->sub(m_it == 0 ? m_buff->m_last : m_it, n);
    } else if (n < 0) {
      *this += -n;
    }
    return *this;
  }

  //! Iterator subtraction.
  iterator operator-(difference_type n) const {
    return iterator<Buff, Traits>(*this) -= n;
  }

  //! Element access operator.
  reference operator[](difference_type n) const { return *(*this + n); }

  // Equality & comparison

  //! Equality.
  template <class Traits0>
  bool operator==(const iterator<Buff, Traits0> &it) const {
    return m_it == it.m_it;
  }

  //! Inequality.
  template <class Traits0>
  bool operator!=(const iterator<Buff, Traits0> &it) const {
    return m_it != it.m_it;
  }

  //! Less.
  template <class Traits0>
  bool operator<(const iterator<Buff, Traits0> &it) const {
    return linearize_pointer(*this) < linearize_pointer(it);
  }

  //! Greater.
  template <class Traits0>
  bool operator>(const iterator<Buff, Traits0> &it) const {
    return it < *this;
  }

  //! Less or equal.
  template <class Traits0>
  bool operator<=(const iterator<Buff, Traits0> &it) const {
    return !(it < *this);
  }

  //! Greater or equal.
  template <class Traits0>
  bool operator>=(const iterator<Buff, Traits0> &it) const {
    return !(*this < it);
  }

  // Helpers

  //! Get a pointer which would point to the same element as the iterator in
  //! case the circular buffer is linearized.
  template <class Traits0>
  typename Traits0::pointer
  linearize_pointer(const iterator<Buff, Traits0> &it) const {
    return it.m_it == 0 ? m_buff->m_buff + m_buff->size()
                        : (it.m_it < m_buff->m_first
                               ? it.m_it + (m_buff->m_end - m_buff->m_first)
                               : m_buff->m_buff + (it.m_it - m_buff->m_first));
  }
};

//! Iterator addition.
template <class Buff, class Traits>
inline iterator<Buff, Traits> operator+(typename Traits::difference_type n,
                                        const iterator<Buff, Traits> &it) {
  return it + n;
}

/*!
    \fn ForwardIterator uninitialized_copy(InputIterator first, InputIterator
   last, ForwardIterator dest) \brief Equivalent of
   <code>std::uninitialized_copy</code> but with explicit specification of value
   type.
*/
template <class InputIterator, class ForwardIterator, class Alloc>
inline ForwardIterator uninitialized_copy(InputIterator first,
                                          InputIterator last,
                                          ForwardIterator dest, Alloc &a) {
  ForwardIterator next = dest;
  {
    try {
      for (; first != last; ++first, ++dest)
        std::allocator_traits<Alloc>::construct(a, std::addressof(*dest),
                                                *first);
    } catch (...) {
      for (; next != dest; ++next)
        std::allocator_traits<Alloc>::destroy(a, std::addressof(*next));
      throw;
    }
  }
  return dest;
}

template <class InputIterator, class ForwardIterator, class Alloc>
ForwardIterator uninitialized_move_if_noexcept_impl(InputIterator first,
                                                    InputIterator last,
                                                    ForwardIterator dest,
                                                    Alloc &a, std::true_type) {
  for (; first != last; ++first, ++dest)
    std::allocator_traits<Alloc>::construct(a, std::addressof(*dest),
                                            std::move(*first));
  return dest;
}

template <class InputIterator, class ForwardIterator, class Alloc>
ForwardIterator uninitialized_move_if_noexcept_impl(InputIterator first,
                                                    InputIterator last,
                                                    ForwardIterator dest,
                                                    Alloc &a, std::false_type) {
  return uninitialized_copy(first, last, dest, a);
}

/*!
    \fn ForwardIterator uninitialized_move_if_noexcept(InputIterator first,
   InputIterator last, ForwardIterator dest) \brief Equivalent of
   <code>std::uninitialized_copy</code> but with explicit specification of value
   type and moves elements if they have noexcept move constructors.
*/
template <class InputIterator, class ForwardIterator, class Alloc>
ForwardIterator uninitialized_move_if_noexcept(InputIterator first,
                                               InputIterator last,
                                               ForwardIterator dest, Alloc &a) {
  typedef typename std::is_nothrow_move_constructible<
      typename std::allocator_traits<Alloc>::value_type>::type tag_t;
  return uninitialized_move_if_noexcept_impl(first, last, dest, a, tag_t());
}

/*!
    \fn void uninitialized_fill_n_with_alloc(ForwardIterator first, Diff n,
   const T& item, Alloc& alloc) \brief Equivalent of
   <code>std::uninitialized_fill_n</code> with allocator.
*/
template <class ForwardIterator, class Diff, class T, class Alloc>
inline void uninitialized_fill_n_with_alloc(ForwardIterator first, Diff n,
                                            const T &item, Alloc &alloc) {
  ForwardIterator next = first;
  {
    try {
      for (; n > 0; ++first, --n)
        std::allocator_traits<Alloc>::construct(alloc, std::addressof(*first),
                                                item);
    } catch (...) {
      for (; next != first; ++next)
        std::allocator_traits<Alloc>::destroy(alloc, std::addressof(*next));
      throw;
    }
  }
}

} // namespace cb_details
} // namespace cb

#if defined(_MSC_VER)
#pragma warning(pop)
#endif
