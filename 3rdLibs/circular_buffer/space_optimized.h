/*
 * @Author: chenjingyu
 * @Date: 2023-11-24 16:14:07
 * @LastEditTime: 2023-11-24 16:20:31
 * @Description: space optimized
 * @FilePath: \CodeBlocks\C++\3rdLibs\circularbuffer\space_optimized.h
 */
#pragma once

#include "circular_buffer.h"
#include <cassert>
#include <type_traits>
#include <utility>


namespace cb {
/*!
    \class circular_buffer_space_optimized
    \brief Space optimized circular buffer container adaptor.
           <code>T</code> must be a copyable class or must have an noexcept move
   constructor and move assignment operator.
*/
template <class T, class Alloc>
class circular_buffer_space_optimized : circular_buffer<T, Alloc> {
public:
  typedef typename circular_buffer<T, Alloc>::value_type value_type;
  typedef typename circular_buffer<T, Alloc>::pointer pointer;
  typedef typename circular_buffer<T, Alloc>::const_pointer const_pointer;
  typedef typename circular_buffer<T, Alloc>::reference reference;
  typedef typename circular_buffer<T, Alloc>::const_reference const_reference;
  typedef typename circular_buffer<T, Alloc>::size_type size_type;
  typedef typename circular_buffer<T, Alloc>::difference_type difference_type;
  typedef typename circular_buffer<T, Alloc>::allocator_type allocator_type;
  typedef typename circular_buffer<T, Alloc>::const_iterator const_iterator;
  typedef typename circular_buffer<T, Alloc>::iterator iterator;
  typedef typename circular_buffer<T, Alloc>::const_reverse_iterator const_reverse_iterator;
  typedef typename circular_buffer<T, Alloc>::reverse_iterator reverse_iterator;
  typedef typename circular_buffer<T, Alloc>::array_range array_range;
  typedef typename circular_buffer<T, Alloc>::const_array_range const_array_range;
  typedef typename circular_buffer<T, Alloc>::param_value_type param_value_type;
  typedef typename circular_buffer<T, Alloc>::rvalue_type rvalue_type;

  /*
   * <pre> is not passed through to html or pdf. So <br> is used in code section
   * below.  Ugly :-( Ideally want a link to capacity_control, but this would
   * require include details and this would expose all the functions in details.
   * There must be a better way of doing this.
   */
  typedef cb_details::capacity_control<size_type> capacity_type;

  // Inherited

  using circular_buffer<T, Alloc>::get_allocator;
  using circular_buffer<T, Alloc>::begin;
  using circular_buffer<T, Alloc>::end;
  using circular_buffer<T, Alloc>::rbegin;
  using circular_buffer<T, Alloc>::rend;
  using circular_buffer<T, Alloc>::at;
  using circular_buffer<T, Alloc>::front;
  using circular_buffer<T, Alloc>::back;
  using circular_buffer<T, Alloc>::array_one;
  using circular_buffer<T, Alloc>::array_two;
  using circular_buffer<T, Alloc>::linearize;
  using circular_buffer<T, Alloc>::is_linearized;
  using circular_buffer<T, Alloc>::rotate;
  using circular_buffer<T, Alloc>::size;
  using circular_buffer<T, Alloc>::max_size;
  using circular_buffer<T, Alloc>::empty;
  using circular_buffer<T, Alloc>::operator[];

private:
  //! The capacity controller of the space optimized circular buffer.
  capacity_type m_capacity_ctrl;

public:
  //! Is the <code>circular_buffer_space_optimized</code> full?
  bool full() const noexcept { return m_capacity_ctrl == size(); }

  /*! \brief Get the maximum number of elements which can be inserted into the
             <code>circular_buffer_space_optimized</code> without overwriting
     any of already stored elements.
  */
  size_type reserve() const noexcept { return m_capacity_ctrl - size(); }

  //! Get the capacity of the <code>circular_buffer_space_optimized</code>.
  const capacity_type &capacity() const noexcept { return m_capacity_ctrl; }

  /*! \brief Change the capacity (and the minimal guaranteed amount of allocated
     memory) of the <code>circular_buffer_space_optimized</code>.
  */
  void set_capacity(const capacity_type &capacity_ctrl) {
    m_capacity_ctrl = capacity_ctrl;
    if (capacity_ctrl < size()) {
      iterator e = end();
      circular_buffer<T, Alloc>::erase(e - (size() - capacity_ctrl), e);
    }
    adjust_min_capacity();
  }

  //! Change the size of the <code>circular_buffer_space_optimized</code>.
  void resize(size_type new_size, param_value_type item = value_type()) {
    if (new_size > size()) {
      if (new_size > m_capacity_ctrl)
        m_capacity_ctrl =
            capacity_type(new_size, m_capacity_ctrl.min_capacity());
      insert(end(), new_size - size(), item);
    } else {
      iterator e = end();
      erase(e - (size() - new_size), e);
    }
  }

  /*! \brief Change the capacity (and the minimal guaranteed amount of allocated
     memory) of the <code>circular_buffer_space_optimized</code>.
  */
  void rset_capacity(const capacity_type &capacity_ctrl) {
    m_capacity_ctrl = capacity_ctrl;
    if (capacity_ctrl < size()) {
      iterator b = begin();
      circular_buffer<T, Alloc>::rerase(b, b + (size() - capacity_ctrl));
    }
    adjust_min_capacity();
  }

  //! Change the size of the <code>circular_buffer_space_optimized</code>.
  void rresize(size_type new_size, param_value_type item = value_type()) {
    if (new_size > size()) {
      if (new_size > m_capacity_ctrl)
        m_capacity_ctrl =
            capacity_type(new_size, m_capacity_ctrl.min_capacity());
      rinsert(begin(), new_size - size(), item);
    } else {
      rerase(begin(), end() - new_size);
    }
  }

  //! Create an empty space optimized circular buffer with zero capacity.
  explicit circular_buffer_space_optimized(
      const allocator_type &alloc = allocator_type()) noexcept
      : circular_buffer<T, Alloc>(0, alloc), m_capacity_ctrl(0) {}

  //! Create an empty space optimized circular buffer with the specified
  //! capacity.
  explicit circular_buffer_space_optimized(
      capacity_type capacity_ctrl,
      const allocator_type &alloc = allocator_type())
      : circular_buffer<T, Alloc>(capacity_ctrl.min_capacity(), alloc),
        m_capacity_ctrl(capacity_ctrl) {}

  /*! \brief Create a full space optimized circular buffer with the specified
     capacity filled with <code>capacity_ctrl.%capacity()</code> copies of
     <code>item</code>.
  */
  circular_buffer_space_optimized(
      capacity_type capacity_ctrl, param_value_type item,
      const allocator_type &alloc = allocator_type())
      : circular_buffer<T, Alloc>(capacity_ctrl.capacity(), item, alloc),
        m_capacity_ctrl(capacity_ctrl) {}

  /*! \brief Create a space optimized circular buffer with the specified
     capacity filled with <code>n</code> copies of <code>item</code>.
  */
  circular_buffer_space_optimized(
      capacity_type capacity_ctrl, size_type n, param_value_type item,
      const allocator_type &alloc = allocator_type())
      : circular_buffer<T, Alloc>(init_capacity(capacity_ctrl, n), n, item,
                                  alloc),
        m_capacity_ctrl(capacity_ctrl) {}

  //! The copy constructor.
  circular_buffer_space_optimized(
      const circular_buffer_space_optimized<T, Alloc> &cb)
      : circular_buffer<T, Alloc>(cb.begin(), cb.end(), cb.get_allocator()),
        m_capacity_ctrl(cb.m_capacity_ctrl) {}

  //! The move constructor.
  circular_buffer_space_optimized(
      circular_buffer_space_optimized<T, Alloc> &&cb) noexcept
      : circular_buffer<T, Alloc>(), m_capacity_ctrl(0) {
    cb.swap(*this);
  }

  //! Create a full space optimized circular buffer filled with a copy of the
  //! range.
  template <class InputIterator>
  circular_buffer_space_optimized(
      InputIterator first, InputIterator last,
      const allocator_type &alloc = allocator_type())
      : circular_buffer<T, Alloc>(first, last, alloc),
        m_capacity_ctrl(circular_buffer<T, Alloc>::capacity()) {}

  /*! \brief Create a space optimized circular buffer with the specified
     capacity (and the minimal guaranteed amount of allocated memory) filled
     with a copy of the range.
  */
  template <class InputIterator>
  circular_buffer_space_optimized(
      capacity_type capacity_ctrl, InputIterator first, InputIterator last,
      const allocator_type &alloc = allocator_type())
      : circular_buffer<T, Alloc>(
            init_capacity(capacity_ctrl, first, last,
                          std::is_integral<InputIterator>()),
            first, last, alloc),
        m_capacity_ctrl(capacity_ctrl) {
    reduce_capacity(std::is_same<typename std::iterator_traits<
                                     InputIterator>::iterator_category::type,
                                 std::input_iterator_tag>());
  }

  //! The assign operator.
  circular_buffer_space_optimized<T, Alloc> &
  operator=(const circular_buffer_space_optimized<T, Alloc> &cb) {
    if (this == &cb)
      return *this;
    circular_buffer<T, Alloc>::assign(cb.begin(), cb.end());
    m_capacity_ctrl = cb.m_capacity_ctrl;
    return *this;
  }

  /*! \brief Move assigns content of <code>cb</code> to <code>*this</code>,
     leaving <code>cb</code> empty. \pre C++ compiler with rvalue references
     support. \post <code>cb.empty()</code> \param cb
     <code>circular_buffer</code> to 'steal' value from. \throws Nothing. \par
     Complexity Constant.
  */
  circular_buffer_space_optimized<T, Alloc> &
  operator=(circular_buffer_space_optimized<T, Alloc> &&cb) noexcept {
    cb.swap(*this); // now `this` holds `cb`
    circular_buffer<T, Alloc>(
        get_allocator()) // temprary that holds initial `cb` allocator
        .swap(cb);       // makes `cb` empty
    return *this;
  }

  //! Assign <code>n</code> items into the space optimized circular buffer.
  void assign(size_type n, param_value_type item) {
    circular_buffer<T, Alloc>::assign(n, item);
    m_capacity_ctrl = capacity_type(n);
  }

  //! Assign <code>n</code> items into the space optimized circular buffer
  //! specifying the capacity.
  void assign(capacity_type capacity_ctrl, size_type n, param_value_type item) {
    assert(capacity_ctrl.capacity() >=
           n); // check for new capacity lower than n
    circular_buffer<T, Alloc>::assign(
        (std::max)(capacity_ctrl.min_capacity(), n), n, item);
    m_capacity_ctrl = capacity_ctrl;
  }

  //! Assign a copy of the range into the space optimized circular buffer.
  template <class InputIterator>
  void assign(InputIterator first, InputIterator last) {
    circular_buffer<T, Alloc>::assign(first, last);
    m_capacity_ctrl = capacity_type(circular_buffer<T, Alloc>::capacity());
  }

  //! Assign a copy of the range into the space optimized circular buffer
  //! specifying the capacity.
  template <class InputIterator>
  void assign(capacity_type capacity_ctrl, InputIterator first,
              InputIterator last) {
    m_capacity_ctrl = capacity_ctrl;
    circular_buffer<T, Alloc>::assign(capacity_ctrl, first, last);
  }

  //! Swap the contents of two space-optimized circular-buffers.
  // Note link does not work right.  Asked on Doxygen forum for advice 23 May
  // 2103.
  void swap(circular_buffer_space_optimized<T, Alloc> &cb) noexcept {
    std::swap(m_capacity_ctrl, cb.m_capacity_ctrl);
    circular_buffer<T, Alloc>::swap(cb);
  }

  //! Insert a new element at the end of the space optimized circular buffer.
  void push_back(param_value_type item) {
    check_low_capacity();
    circular_buffer<T, Alloc>::push_back(item);
  }

  //! Insert a new element at the end of the space optimized circular buffer.
  void push_back(rvalue_type item) {
    check_low_capacity();
    circular_buffer<T, Alloc>::push_back(std::move(item));
  }

  //! Insert a new element at the end of the space optimized circular buffer.
  void push_back() {
    check_low_capacity();
    circular_buffer<T, Alloc>::push_back();
  }

  //! Insert a new element at the beginning of the space optimized circular
  //! buffer.
  void push_front(param_value_type item) {
    check_low_capacity();
    circular_buffer<T, Alloc>::push_front(item);
  }

  //! Insert a new element at the beginning of the space optimized circular
  //! buffer.
  void push_front(rvalue_type item) {
    check_low_capacity();
    circular_buffer<T, Alloc>::push_front(std::move(item));
  }

  //! Insert a new element at the beginning of the space optimized circular
  //! buffer.
  void push_front() {
    check_low_capacity();
    circular_buffer<T, Alloc>::push_front();
  }

  //! Remove the last element from the space optimized circular buffer.
  void pop_back() {
    circular_buffer<T, Alloc>::pop_back();
    check_high_capacity();
  }

  //! Remove the first element from the space optimized circular buffer.
  void pop_front() {
    circular_buffer<T, Alloc>::pop_front();
    check_high_capacity();
  }

  //! Insert an element at the specified position.
  iterator insert(iterator pos, param_value_type item) {
    size_type index = pos - begin();
    check_low_capacity();
    return circular_buffer<T, Alloc>::insert(begin() + index, item);
  }

  //! Insert an element at the specified position.
  iterator insert(iterator pos, rvalue_type item) {
    size_type index = pos - begin();
    check_low_capacity();
    return circular_buffer<T, Alloc>::insert(begin() + index, std::move(item));
  }

  //! Insert an element at the specified position.
  iterator insert(iterator pos) {
    size_type index = pos - begin();
    check_low_capacity();
    return circular_buffer<T, Alloc>::insert(begin() + index);
  }

  //! Insert <code>n</code> copies of the <code>item</code> at the specified
  //! position.
  void insert(iterator pos, size_type n, param_value_type item) {
    size_type index = pos - begin();
    check_low_capacity(n);
    circular_buffer<T, Alloc>::insert(begin() + index, n, item);
  }

  //! Insert the range <code>[first, last)</code> at the specified position.
  template <class InputIterator>
  void insert(iterator pos, InputIterator first, InputIterator last) {
    insert(pos, first, last, std::is_integral<InputIterator>());
  }

  //! Insert an element before the specified position.
  iterator rinsert(iterator pos, param_value_type item) {
    size_type index = pos - begin();
    check_low_capacity();
    return circular_buffer<T, Alloc>::rinsert(begin() + index, item);
  }

  //! Insert an element before the specified position.
  iterator rinsert(iterator pos, rvalue_type item) {
    size_type index = pos - begin();
    check_low_capacity();
    return circular_buffer<T, Alloc>::rinsert(begin() + index, std::move(item));
  }

  //! Insert an element before the specified position.
  iterator rinsert(iterator pos) {
    size_type index = pos - begin();
    check_low_capacity();
    return circular_buffer<T, Alloc>::rinsert(begin() + index);
  }

  //! Insert <code>n</code> copies of the <code>item</code> before the specified
  //! position.
  void rinsert(iterator pos, size_type n, param_value_type item) {
    size_type index = pos - begin();
    check_low_capacity(n);
    circular_buffer<T, Alloc>::rinsert(begin() + index, n, item);
  }

  //! Insert the range <code>[first, last)</code> before the specified position.
  template <class InputIterator>
  void rinsert(iterator pos, InputIterator first, InputIterator last) {
    rinsert(pos, first, last, std::is_integral<InputIterator>());
  }

  //! Remove an element at the specified position.
  iterator erase(iterator pos) {
    iterator it = circular_buffer<T, Alloc>::erase(pos);
    size_type index = it - begin();
    check_high_capacity();
    return begin() + index;
  }

  //! Erase the range <code>[first, last)</code>.
  iterator erase(iterator first, iterator last) {
    iterator it = circular_buffer<T, Alloc>::erase(first, last);
    size_type index = it - begin();
    check_high_capacity();
    return begin() + index;
  }

  //! Remove an element at the specified position.
  iterator rerase(iterator pos) {
    iterator it = circular_buffer<T, Alloc>::rerase(pos);
    size_type index = it - begin();
    check_high_capacity();
    return begin() + index;
  }

  //! Erase the range <code>[first, last)</code>.
  iterator rerase(iterator first, iterator last) {
    iterator it = circular_buffer<T, Alloc>::rerase(first, last);
    size_type index = it - begin();
    check_high_capacity();
    return begin() + index;
  }

  //! Remove all stored elements from the space optimized circular buffer.
  void clear() { erase(begin(), end()); }

private:
  // Helper methods

  //! Adjust the amount of allocated memory.
  void adjust_min_capacity() {
    if (m_capacity_ctrl.min_capacity() > circular_buffer<T, Alloc>::capacity())
      circular_buffer<T, Alloc>::set_capacity(m_capacity_ctrl.min_capacity());
    else
      check_high_capacity();
  }

  //! Ensure the reserve for possible growth up.
  size_type ensure_reserve(size_type new_capacity,
                           size_type buffer_size) const {
    if (buffer_size + new_capacity / 5 >= new_capacity)
      new_capacity *= 2; // ensure at least 20% reserve
    if (new_capacity > m_capacity_ctrl)
      return m_capacity_ctrl;
    return new_capacity;
  }

  //! Check for low capacity.
  /*
      \post If the capacity is low it will be increased.
  */
  void check_low_capacity(size_type n = 1) {
    size_type new_size = size() + n;
    size_type new_capacity = circular_buffer<T, Alloc>::capacity();
    if (new_size > new_capacity) {
      if (new_capacity == 0)
        new_capacity = 1;
      for (; new_size > new_capacity; new_capacity *= 2) {
      }
      circular_buffer<T, Alloc>::set_capacity(
          ensure_reserve(new_capacity, new_size));
    }
  }

  //! Check for high capacity.
  /*
      \post If the capacity is high it will be decreased.
  */
  void check_high_capacity() {
    size_type new_capacity = circular_buffer<T, Alloc>::capacity();
    while (new_capacity / 3 >=
           size()) { // (new_capacity / 3) -> avoid oscillations
      new_capacity /= 2;
      if (new_capacity <= m_capacity_ctrl.min_capacity()) {
        new_capacity = m_capacity_ctrl.min_capacity();
        break;
      }
    }
    circular_buffer<T, Alloc>::set_capacity(
        ensure_reserve(new_capacity, size()));
  }

  //! Specialized method for reducing the capacity.
  void reduce_capacity(const std::true_type &) {
    circular_buffer<T, Alloc>::set_capacity(
        (std::max)(m_capacity_ctrl.min_capacity(), size()));
  }

  //! Specialized method for reducing the capacity.
  void reduce_capacity(const std::false_type &) {}

  //! Determine the initial capacity.
  static size_type init_capacity(const capacity_type &capacity_ctrl,
                                 size_type n) {
    assert(capacity_ctrl.capacity() >= n); // check for capacity lower than n
    return (std::max)(capacity_ctrl.min_capacity(), n);
  }

  //! Specialized method for determining the initial capacity.
  template <class IntegralType>
  static size_type init_capacity(const capacity_type &capacity_ctrl,
                                 IntegralType n, IntegralType,
                                 const std::true_type &) {
    return init_capacity(capacity_ctrl, static_cast<size_type>(n));
  }

  //! Specialized method for determining the initial capacity.
  template <class Iterator>
  static size_type init_capacity(const capacity_type &capacity_ctrl,
                                 Iterator first, Iterator last,
                                 const std::false_type &) {
    static_assert((std::is_convertible<
                      typename std::iterator_traits<Iterator>::value_type,
                      value_type>::value),
                  "Invalid iterator type");
    return init_capacity(
        capacity_ctrl, first, last,
        Iterator::template iterator_category<Iterator>::type());
  }

  //! Specialized method for determining the initial capacity.
  template <class InputIterator>
  static size_type init_capacity(const capacity_type &capacity_ctrl,
                                 InputIterator, InputIterator,
                                 const std::input_iterator_tag &) {
    return capacity_ctrl.capacity();
  }

  //! Specialized method for determining the initial capacity.
  template <class ForwardIterator>
  static size_type init_capacity(const capacity_type &capacity_ctrl,
                                 ForwardIterator first, ForwardIterator last,
                                 const std::forward_iterator_tag &) {
    assert(std::distance(first, last) >= 0); // check for wrong range
    return (std::max)(
        capacity_ctrl.min_capacity(),
        (std::min)(capacity_ctrl.capacity(),
                   static_cast<size_type>(std::distance(first, last))));
  }

  //! Specialized insert method.
  template <class IntegralType>
  void insert(const iterator &pos, IntegralType n, IntegralType item,
              const std::true_type &) {
    insert(pos, static_cast<size_type>(n), static_cast<value_type>(item));
  }

  //! Specialized insert method.
  template <class Iterator>
  void insert(const iterator &pos, Iterator first, Iterator last,
              const std::false_type &) {
    size_type index = pos - begin();
    check_low_capacity(std::distance(first, last));
    circular_buffer<T, Alloc>::insert(begin() + index, first, last);
  }

  //! Specialized rinsert method.
  template <class IntegralType>
  void rinsert(const iterator &pos, IntegralType n, IntegralType item,
               const std::true_type &) {
    rinsert(pos, static_cast<size_type>(n), static_cast<value_type>(item));
  }

  //! Specialized rinsert method.
  template <class Iterator>
  void rinsert(const iterator &pos, Iterator first, Iterator last,
               const std::false_type &) {
    size_type index = pos - begin();
    check_low_capacity(std::distance(first, last));
    circular_buffer<T, Alloc>::rinsert(begin() + index, first, last);
  }
};

// Non-member functions

//! Test two space optimized circular buffers for equality.
template <class T, class Alloc>
inline bool operator==(const circular_buffer_space_optimized<T, Alloc> &lhs,
                       const circular_buffer_space_optimized<T, Alloc> &rhs) {
  return lhs.size() == rhs.size() &&
         std::equal(lhs.begin(), lhs.end(), rhs.begin());
}

//! Lexicographical comparison.
template <class T, class Alloc>
inline bool operator<(const circular_buffer_space_optimized<T, Alloc> &lhs,
                      const circular_buffer_space_optimized<T, Alloc> &rhs) {
  return std::lexicographical_compare(lhs.begin(), lhs.end(), rhs.begin(),
                                      rhs.end());
}

//! Test two space optimized circular buffers for non-equality.
template <class T, class Alloc>
inline bool operator!=(const circular_buffer_space_optimized<T, Alloc> &lhs,
                       const circular_buffer_space_optimized<T, Alloc> &rhs) {
  return !(lhs == rhs);
}

//! Lexicographical comparison.
template <class T, class Alloc>
inline bool operator>(const circular_buffer_space_optimized<T, Alloc> &lhs,
                      const circular_buffer_space_optimized<T, Alloc> &rhs) {
  return rhs < lhs;
}

//! Lexicographical comparison.
template <class T, class Alloc>
inline bool operator<=(const circular_buffer_space_optimized<T, Alloc> &lhs,
                       const circular_buffer_space_optimized<T, Alloc> &rhs) {
  return !(rhs < lhs);
}

//! Lexicographical comparison.
template <class T, class Alloc>
inline bool operator>=(const circular_buffer_space_optimized<T, Alloc> &lhs,
                       const circular_buffer_space_optimized<T, Alloc> &rhs) {
  return !(lhs < rhs);
}

//! Swap the contents of two space optimized circular buffers.
template <class T, class Alloc>
inline void swap(circular_buffer_space_optimized<T, Alloc> &lhs,
                 circular_buffer_space_optimized<T, Alloc> &rhs) noexcept {
  lhs.swap(rhs);
}
} // namespace cb
