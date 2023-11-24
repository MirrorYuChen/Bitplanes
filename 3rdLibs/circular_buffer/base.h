/*
 * @Author: chenjingyu
 * @Date: 2023-11-24 15:57:38
 * @LastEditTime: 2023-11-24 16:11:21
 * @Description: Base
 * @FilePath: \CodeBlocks\C++\3rdLibs\circularbuffer\base.h
 */
#pragma once

#include <algorithm>
#include <cassert>
#include <deque>
#include <iterator>
#include <limits>
#include <memory>
#include <stdexcept>
#include <type_traits>
#include <utility>

#include "circular_buffer.h"

namespace cb {
template <class T, class Alloc> class circular_buffer {
public:
  //! The type of this <code>circular_buffer</code>.
  typedef circular_buffer<T, Alloc> this_type;

  //! The type of elements stored in the <code>circular_buffer</code>.
  typedef typename std::allocator_traits<Alloc>::value_type value_type;

  //! A pointer to an element.
  typedef typename std::allocator_traits<Alloc>::pointer pointer;

  //! A const pointer to the element.
  typedef typename std::allocator_traits<Alloc>::const_pointer const_pointer;

  //! A reference to an element.
  typedef typename std::allocator_traits<Alloc>::value_type &reference;

  //! A const reference to an element.
  typedef const typename std::allocator_traits<Alloc>::value_type
      &const_reference;

  //! The distance type.
  /*!
      (A signed integral type used to represent the distance between two
     iterators.)
  */
  typedef
      typename std::allocator_traits<Alloc>::difference_type difference_type;

  //! The size type.
  /*!
      (An unsigned integral type that can represent any non-negative value of
     the container's distance type.)
  */
  typedef typename std::allocator_traits<Alloc>::size_type size_type;

  //! The type of an allocator used in the <code>circular_buffer</code>.
  typedef Alloc allocator_type;

  //! A const (random access) iterator used to iterate through the
  //! <code>circular_buffer</code>.
  typedef cb_details::iterator<
      circular_buffer<T, Alloc>,
      cb_details::const_traits<std::allocator_traits<Alloc>>>
      const_iterator;

  //! A (random access) iterator used to iterate through the
  //! <code>circular_buffer</code>.
  typedef cb_details::iterator<
      circular_buffer<T, Alloc>,
      cb_details::nonconst_traits<std::allocator_traits<Alloc>>>
      iterator;

  //! A const iterator used to iterate backwards through a
  //! <code>circular_buffer</code>.
  typedef std::reverse_iterator<const_iterator> const_reverse_iterator;

  //! An iterator used to iterate backwards through a
  //! <code>circular_buffer</code>.
  typedef std::reverse_iterator<iterator> reverse_iterator;

  //! An array range.
  typedef std::pair<pointer, size_type> array_range;

  //! A range of a const array.
  typedef std::pair<const_pointer, size_type> const_array_range;

  //! The capacity type.
  /*!
      (Same as <code>size_type</code> - defined for consistency with the  __cbso
     class.

  */
  // <a
  // href="space_optimized.html"><code>circular_buffer_space_optimized</code></a>.)
  typedef size_type capacity_type;

  //! A type representing the "best" way to pass the value_type to a method.
  typedef const value_type &param_value_type;

  //! A type representing rvalue from param type.
  //! On compilers without rvalue references support this type is the
  //! Boost.Moves type used for emulation.
  typedef value_type &&rvalue_type;

private:
  //! The internal buffer used for storing elements in the circular buffer.
  pointer m_buff;

  //! The internal buffer's end (end of the storage space).
  pointer m_end;

  //! The virtual beginning of the circular buffer.
  pointer m_first;

  //! The virtual end of the circular buffer (one behind the last element).
  pointer m_last;

  //! The number of items currently stored in the circular buffer.
  size_type m_size;

  //! The allocator.
  allocator_type m_alloc;

  template <class Buff, class Traits> friend struct cb_details::iterator;

public:
  //! Get the allocator.
  allocator_type get_allocator() const noexcept { return m_alloc; }

  //! Get the allocator reference.
  allocator_type &get_allocator() noexcept { return m_alloc; }

  //! Get the iterator pointing to the beginning of the
  //! <code>circular_buffer</code>.
  iterator begin() noexcept { return iterator(this, empty() ? 0 : m_first); }

  //! Get the iterator pointing to the end of the <code>circular_buffer</code>.
  iterator end() noexcept { return iterator(this, 0); }

  //! Get the const iterator pointing to the beginning of the
  //! <code>circular_buffer</code>.
  const_iterator begin() const noexcept {
    return const_iterator(this, empty() ? 0 : m_first);
  }

  //! Get the const iterator pointing to the end of the
  //! <code>circular_buffer</code>.
  const_iterator end() const noexcept { return const_iterator(this, 0); }

  //! Get the iterator pointing to the beginning of the "reversed"
  //! <code>circular_buffer</code>.
  reverse_iterator rbegin() noexcept { return reverse_iterator(end()); }

  //! Get the iterator pointing to the end of the "reversed"
  //! <code>circular_buffer</code>.
  reverse_iterator rend() noexcept { return reverse_iterator(begin()); }

  //! Get the const iterator pointing to the beginning of the "reversed"
  //! <code>circular_buffer</code>.
  const_reverse_iterator rbegin() const noexcept {
    return const_reverse_iterator(end());
  }

  //! Get the const iterator pointing to the end of the "reversed"
  //! <code>circular_buffer</code>.
  const_reverse_iterator rend() const noexcept {
    return const_reverse_iterator(begin());
  }

  //! Get the element at the <code>index</code> position.
  reference operator[](size_type index) {
    assert(index < size()); // check for invalid index
    return *add(m_first, index);
  }

  //! Get the element at the <code>index</code> position.
  const_reference operator[](size_type index) const {
    assert(index < size()); // check for invalid index
    return *add(m_first, index);
  }

  //! Get the element at the <code>index</code> position.
  reference at(size_type index) {
    check_position(index);
    return (*this)[index];
  }

  //! Get the element at the <code>index</code> position.
  const_reference at(size_type index) const {
    check_position(index);
    return (*this)[index];
  }

  //! Get the first element.
  reference front() {
    assert(!empty()); // check for empty buffer (front element not available)
    return *m_first;
  }

  //! Get the last element.
  reference back() {
    assert(!empty()); // check for empty buffer (back element not available)
    return *((m_last == m_buff ? m_end : m_last) - 1);
  }

  //! Get the first element.
  const_reference front() const {
    assert(!empty()); // check for empty buffer (front element not available)
    return *m_first;
  }

  //! Get the last element.
  const_reference back() const {
    assert(!empty()); // check for empty buffer (back element not available)
    return *((m_last == m_buff ? m_end : m_last) - 1);
  }

  //! Get the first continuous array of the internal buffer.
  array_range array_one() {
    return array_range(
        m_first, (m_last <= m_first && !empty() ? m_end : m_last) - m_first);
  }

  //! Get the second continuous array of the internal buffer.
  array_range array_two() {
    return array_range(m_buff,
                       m_last <= m_first && !empty() ? m_last - m_buff : 0);
  }

  //! Get the first continuous array of the internal buffer.
  const_array_range array_one() const {
    return const_array_range(
        m_first, (m_last <= m_first && !empty() ? m_end : m_last) - m_first);
  }

  //! Get the second continuous array of the internal buffer.
  const_array_range array_two() const {
    return const_array_range(
        m_buff, m_last <= m_first && !empty() ? m_last - m_buff : 0);
  }

  //! Linearize the internal buffer into a continuous array.
  pointer linearize() {
    if (empty())
      return 0;
    if (m_first < m_last || m_last == m_buff)
      return m_first;
    pointer src = m_first;
    pointer dest = m_buff;
    size_type moved = 0;
    size_type constructed = 0;
    {
      try {
        for (pointer first = m_first; dest < src; src = first) {
          for (size_type ii = 0; src < m_end; ++src, ++dest, ++moved, ++ii) {
            if (moved == size()) {
              first = dest;
              break;
            }
            if (dest == first) {
              first += ii;
              break;
            }
            if (is_uninitialized(dest)) {
              std::allocator_traits<Alloc>::construct(
                  m_alloc, std::addressof(*dest), std::move_if_noexcept(*src));
              ++constructed;
            } else {
              value_type tmp = std::move_if_noexcept(*src);
              replace(src, std::move_if_noexcept(*dest));
              replace(dest, std::move(tmp));
            }
          }
        }
      } catch (...) {
        m_last += constructed;
        m_size += constructed;
        throw;
      }
    }
    for (src = m_end - constructed; src < m_end; ++src)
      destroy_item(src);
    m_first = m_buff;
    m_last = add(m_buff, size());
    return m_buff;
  }

  //! Is the <code>circular_buffer</code> linearized?
  bool is_linearized() const noexcept {
    return m_first < m_last || m_last == m_buff;
  }

  //! Rotate elements in the <code>circular_buffer</code>.
  void rotate(const_iterator new_begin) {
    assert(new_begin.is_valid(
        this)); // check for uninitialized or invalidated iterator
    assert(new_begin.m_it != 0); // check for iterator pointing to end()
    if (full()) {
      m_first = m_last = const_cast<pointer>(new_begin.m_it);
    } else {
      difference_type m = end() - new_begin;
      difference_type n = new_begin - begin();
      if (m < n) {
        for (; m > 0; --m) {
          push_front(std::move_if_noexcept(back()));
          pop_back();
        }
      } else {
        for (; n > 0; --n) {
          push_back(std::move_if_noexcept(front()));
          pop_front();
        }
      }
    }
  }

  // Size and capacity

  //! Get the number of elements currently stored in the
  size_type size() const noexcept { return m_size; }

  /*! \brief Get the largest possible size or capacity of the
     <code>circular_buffer</code>. (It depends on allocator's %max_size()).
  */
  size_type max_size() const noexcept {
    return (
        std::min<size_type>)(std::allocator_traits<Alloc>::max_size(m_alloc),
                             (std::numeric_limits<difference_type>::max)());
  }

  //! Is the <code>circular_buffer</code> empty?
  bool empty() const noexcept { return size() == 0; }

  //! Is the <code>circular_buffer</code> full?
  bool full() const noexcept { return capacity() == size(); }

  /*! \brief Get the maximum number of elements which can be inserted into the
     <code>circular_buffer</code> without overwriting any of already stored
     elements.
  */
  size_type reserve() const noexcept { return capacity() - size(); }

  //! Get the capacity of the <code>circular_buffer</code>.
  capacity_type capacity() const noexcept { return m_end - m_buff; }

  //! Change the capacity of the <code>circular_buffer</code>.
  void set_capacity(capacity_type new_capacity) {
    if (new_capacity == capacity())
      return;
    pointer buff = allocate(new_capacity);
    iterator b = begin();
    {
      try {
        reset(buff,
              cb_details::uninitialized_move_if_noexcept(
                  b, b + (std::min)(new_capacity, size()), buff, m_alloc),
              new_capacity);
      } catch (...) {
        deallocate(buff, new_capacity);
        throw;
      }
    }
  }

  //! Change the size of the <code>circular_buffer</code>.
  void resize(size_type new_size, param_value_type item = value_type()) {
    if (new_size > size()) {
      if (new_size > capacity())
        set_capacity(new_size);
      insert(end(), new_size - size(), item);
    } else {
      iterator e = end();
      erase(e - (size() - new_size), e);
    }
  }

  //! Change the capacity of the <code>circular_buffer</code>.
  void rset_capacity(capacity_type new_capacity) {
    if (new_capacity == capacity())
      return;
    pointer buff = allocate(new_capacity);
    iterator e = end();
    {
      try {
        reset(buff,
              cb_details::uninitialized_move_if_noexcept(
                  e - (std::min)(new_capacity, size()), e, buff, m_alloc),
              new_capacity);
      } catch (...) {
        deallocate(buff, new_capacity);
        throw;
      }
    }
  }

  //! Change the size of the <code>circular_buffer</code>.
  void rresize(size_type new_size, param_value_type item = value_type()) {
    if (new_size > size()) {
      if (new_size > capacity())
        set_capacity(new_size);
      rinsert(begin(), new_size - size(), item);
    } else {
      rerase(begin(), end() - new_size);
    }
  }

  // Construction/Destruction

  //! Create an empty <code>circular_buffer</code> with zero capacity.
  explicit circular_buffer(
      const allocator_type &alloc = allocator_type()) noexcept
      : m_buff(0), m_end(0), m_first(0), m_last(0), m_size(0), m_alloc(alloc) {}

  //! Create an empty <code>circular_buffer</code> with the specified capacity.
  explicit circular_buffer(capacity_type buffer_capacity,
                           const allocator_type &alloc = allocator_type())
      : m_size(0), m_alloc(alloc) {
    initialize_buffer(buffer_capacity);
    m_first = m_last = m_buff;
  }

  /*! \brief Create a full <code>circular_buffer</code> with the specified
     capacity and filled with <code>n</code> copies of <code>item</code>.
  */
  circular_buffer(size_type n, param_value_type item,
                  const allocator_type &alloc = allocator_type())
      : m_size(n), m_alloc(alloc) {
    initialize_buffer(n, item);
    m_first = m_last = m_buff;
  }

  /*! \brief Create a <code>circular_buffer</code> with the specified capacity
     and filled with <code>n</code> copies of <code>item</code>.
  */
  circular_buffer(capacity_type buffer_capacity, size_type n,
                  param_value_type item,
                  const allocator_type &alloc = allocator_type())
      : m_size(n), m_alloc(alloc) {
    assert(buffer_capacity >= size()); // check for capacity lower than size
    initialize_buffer(buffer_capacity, item);
    m_first = m_buff;
    m_last = buffer_capacity == n ? m_buff : m_buff + n;
  }

  //! The copy constructor.
  circular_buffer(const circular_buffer<T, Alloc> &cb)
      : m_size(cb.size()), m_alloc(cb.get_allocator()) {
    initialize_buffer(cb.capacity());
    m_first = m_buff;
    {
      try {
        m_last = cb_details::uninitialized_copy(cb.begin(), cb.end(), m_buff,
                                                m_alloc);
      } catch (...) {
        deallocate(m_buff, cb.capacity());
        throw;
      }
    }
    if (m_last == m_end)
      m_last = m_buff;
  }

  //! The move constructor.
  circular_buffer(circular_buffer<T, Alloc> &&cb) noexcept
      : m_buff(0), m_end(0), m_first(0), m_last(0), m_size(0),
        m_alloc(cb.get_allocator()) {
    cb.swap(*this);
  }

  //! Create a full <code>circular_buffer</code> filled with a copy of the
  //! range.
  template <class InputIterator>
  circular_buffer(InputIterator first, InputIterator last,
                  const allocator_type &alloc = allocator_type())
      : m_alloc(alloc) {
    initialize(first, last, std::is_integral<InputIterator>());
  }

  //! Create a <code>circular_buffer</code> with the specified capacity and
  //! filled with a copy of the range.
  template <class InputIterator>
  circular_buffer(capacity_type buffer_capacity, InputIterator first,
                  InputIterator last,
                  const allocator_type &alloc = allocator_type())
      : m_alloc(alloc) {
    initialize(buffer_capacity, first, last, std::is_integral<InputIterator>());
  }

  //! The destructor.
  ~circular_buffer() noexcept { destroy(); }

public:
  //! The assign operator.
  circular_buffer<T, Alloc> &operator=(const circular_buffer<T, Alloc> &cb) {
    if (this == &cb)
      return *this;
    pointer buff = allocate(cb.capacity());
    {
      try {
        reset(
            buff,
            cb_details::uninitialized_copy(cb.begin(), cb.end(), buff, m_alloc),
            cb.capacity());
      } catch (...) {
        deallocate(buff, cb.capacity());
        throw;
      }
    }
    return *this;
  }

  /*! \brief Move assigns content of <code>cb</code> to <code>*this</code>,
   */
  circular_buffer<T, Alloc> &
  operator=(circular_buffer<T, Alloc> &&cb) noexcept {
    cb.swap(*this); // now `this` holds `cb`
    circular_buffer<T, Alloc>(
        get_allocator()) // temprary that holds initial `cb` allocator
        .swap(cb);       // makes `cb` empty
    return *this;
  }

  //! Assign <code>n</code> items into the <code>circular_buffer</code>.
  void assign(size_type n, param_value_type item) {
    assign_n(n, n,
             cb_details::assign_n<param_value_type, allocator_type>(n, item,
                                                                    m_alloc));
  }

  //! Assign <code>n</code> items into the <code>circular_buffer</code>
  //! specifying the capacity.
  void assign(capacity_type buffer_capacity, size_type n,
              param_value_type item) {
    assert(buffer_capacity >= n); // check for new capacity lower than n
    assign_n(buffer_capacity, n,
             cb_details::assign_n<param_value_type, allocator_type>(n, item,
                                                                    m_alloc));
  }

  //! Assign a copy of the range into the <code>circular_buffer</code>.
  template <class InputIterator>
  void assign(InputIterator first, InputIterator last) {
    assign(first, last, std::is_integral<InputIterator>());
  }

  //! Assign a copy of the range into the <code>circular_buffer</code>
  //! specifying the capacity.
  template <class InputIterator>
  void assign(capacity_type buffer_capacity, InputIterator first,
              InputIterator last) {
    assign(buffer_capacity, first, last, std::is_integral<InputIterator>());
  }

  //! Swap the contents of two <code>circular_buffer</code>s.
  void swap(circular_buffer<T, Alloc> &cb) noexcept {
    std::swap(m_buff, cb.m_buff);
    std::swap(m_end, cb.m_end);
    std::swap(m_first, cb.m_first);
    std::swap(m_last, cb.m_last);
    std::swap(m_size, cb.m_size);
  }

  // push and pop
private:
  template <class ValT> void push_back_impl(ValT item) {
    if (full()) {
      if (empty())
        return;
      replace(m_last, static_cast<ValT>(item));
      increment(m_last);
      m_first = m_last;
    } else {
      std::allocator_traits<Alloc>::construct(m_alloc, std::addressof(*m_last),
                                              static_cast<ValT>(item));
      increment(m_last);
      ++m_size;
    }
  }

  template <class ValT> void push_front_impl(ValT item) {
    {
      try {
        if (full()) {
          if (empty())
            return;
          decrement(m_first);
          replace(m_first, static_cast<ValT>(item));
          m_last = m_first;
        } else {
          decrement(m_first);
          std::allocator_traits<Alloc>::construct(
              m_alloc, std::addressof(*m_first), static_cast<ValT>(item));
          ++m_size;
        }
      } catch (...) {
        increment(m_first);
        throw;
      }
    }
  }

public:
  //! Insert a new element at the end of the <code>circular_buffer</code>.
  void push_back(param_value_type item) {
    push_back_impl<param_value_type>(item);
  }

  //! Insert a new element at the end of the <code>circular_buffer</code> using
  void push_back(rvalue_type item) {
    push_back_impl<rvalue_type>(std::move(item));
  }

  //! Insert a new default-constructed element at the end of the
  void push_back() {
    value_type temp;
    push_back(std::move(temp));
  }

  //! Insert a new element at the beginning of the <code>circular_buffer</code>.
  void push_front(param_value_type item) {
    push_front_impl<param_value_type>(item);
  }

  //! Insert a new element at the beginning of the <code>circular_buffer</code>
  //! using rvalue references or rvalues references emulation.
  void push_front(rvalue_type item) {
    push_front_impl<rvalue_type>(std::move(item));
  }

  //! Insert a new default-constructed element at the beginning of the
  void push_front() {
    value_type temp;
    push_front(std::move(temp));
  }

  //! Remove the last element from the <code>circular_buffer</code>.
  void pop_back() {
    assert(!empty()); // check for empty buffer (back element not available)
    decrement(m_last);
    destroy_item(m_last);
    --m_size;
  }

  //! Remove the first element from the <code>circular_buffer</code>.
  void pop_front() {
    assert(!empty()); // check for empty buffer (front element not available)
    destroy_item(m_first);
    increment(m_first);
    --m_size;
  }

private:
  template <class ValT> iterator insert_impl(iterator pos, ValT item) {
    assert(
        pos.is_valid(this)); // check for uninitialized or invalidated iterator
    iterator b = begin();
    if (full() && pos == b)
      return b;
    return insert_item<ValT>(pos, static_cast<ValT>(item));
  }

public:
  // Insert

  //! Insert an element at the specified position.
  iterator insert(iterator pos, param_value_type item) {
    return insert_impl<param_value_type>(pos, item);
  }

  //! Insert an element at the specified position.
  iterator insert(iterator pos, rvalue_type item) {
    return insert_impl<rvalue_type>(pos, std::move(item));
  }

  //! Insert a default-constructed element at the specified position.
  iterator insert(iterator pos) {
    value_type temp;
    return insert(pos, std::move(temp));
  }

  //! Insert <code>n</code> copies of the <code>item</code> at the specified
  //! position.
  void insert(iterator pos, size_type n, param_value_type item) {
    if (n == 0)
      return;
    size_type copy = capacity() - (end() - pos);
    if (copy == 0)
      return;
    if (n > copy)
      n = copy;
    insert_n(pos, n,
             cb_details::item_wrapper<const_pointer, param_value_type>(item));
  }

  //! Insert the range <code>[first, last)</code> at the specified position.
  template <class InputIterator>
  void insert(iterator pos, InputIterator first, InputIterator last) {
    assert(
        pos.is_valid(this)); // check for uninitialized or invalidated iterator
    insert(pos, first, last, std::is_integral<InputIterator>());
  }

private:
  template <class ValT> iterator rinsert_impl(iterator pos, ValT item) {
    assert(
        pos.is_valid(this)); // check for uninitialized or invalidated iterator
    if (full() && pos.m_it == 0)
      return end();
    if (pos == begin()) {
      {
        try {
          decrement(m_first);
          construct_or_replace(!full(), m_first, static_cast<ValT>(item));
        } catch (...) {
          increment(m_first);
          throw;
        }
      }
      pos.m_it = m_first;
    } else {
      pointer src = m_first;
      pointer dest = m_first;
      decrement(dest);
      pos.m_it = map_pointer(pos.m_it);
      bool construct = !full();
      {
        try {
          while (src != pos.m_it) {
            construct_or_replace(construct, dest, std::move_if_noexcept(*src));
            increment(src);
            increment(dest);
            construct = false;
          }
          decrement(pos.m_it);
          replace(pos.m_it, static_cast<ValT>(item));
        } catch (...) {
          if (!construct && !full()) {
            decrement(m_first);
            ++m_size;
          }
          throw;
        }
      }
      decrement(m_first);
    }
    if (full())
      m_last = m_first;
    else
      ++m_size;
    return iterator(this, pos.m_it);
  }

public:
  //! Insert an element before the specified position.
  iterator rinsert(iterator pos, param_value_type item) {
    return rinsert_impl<param_value_type>(pos, item);
  }

  //! Insert an element before the specified position.
  iterator rinsert(iterator pos, rvalue_type item) {
    return rinsert_impl<rvalue_type>(pos, std::move(item));
  }

  //! Insert an element before the specified position.
  iterator rinsert(iterator pos) {
    value_type temp;
    return rinsert(pos, std::move(temp));
  }

  //! Insert <code>n</code> copies of the <code>item</code> before the specified
  //! position.
  void rinsert(iterator pos, size_type n, param_value_type item) {
    assert(
        pos.is_valid(this)); // check for uninitialized or invalidated iterator
    rinsert_n(pos, n,
              cb_details::item_wrapper<const_pointer, param_value_type>(item));
  }

  //! Insert the range <code>[first, last)</code> before the specified position.
  template <class InputIterator>
  void rinsert(iterator pos, InputIterator first, InputIterator last) {
    assert(
        pos.is_valid(this)); // check for uninitialized or invalidated iterator
    rinsert(pos, first, last, std::is_integral<InputIterator>());
  }

  // Erase

  //! Remove an element at the specified position.
  iterator erase(iterator pos) {
    assert(
        pos.is_valid(this)); // check for uninitialized or invalidated iterator
    assert(pos.m_it != 0);   // check for iterator pointing to end()
    pointer next = pos.m_it;
    increment(next);
    for (pointer p = pos.m_it; next != m_last; p = next, increment(next))
      replace(p, std::move_if_noexcept(*next));
    decrement(m_last);
    destroy_item(m_last);
    --m_size;
    return m_last == pos.m_it ? end() : pos;
  }

  //! Erase the range <code>[first, last)</code>.
  iterator erase(iterator first, iterator last) {
    assert(first <= last); // check for wrong range
    if (first == last)
      return first;
    pointer p = first.m_it;
    while (last.m_it != 0)
      replace((first++).m_it, std::move_if_noexcept(*last++));
    do {
      decrement(m_last);
      destroy_item(m_last);
      --m_size;
    } while (m_last != first.m_it);
    return m_last == p ? end() : iterator(this, p);
  }

  //! Remove an element at the specified position.
  iterator rerase(iterator pos) {
    assert(
        pos.is_valid(this)); // check for uninitialized or invalidated iterator
    assert(pos.m_it != 0);   // check for iterator pointing to end()
    pointer prev = pos.m_it;
    pointer p = prev;
    for (decrement(prev); p != m_first; p = prev, decrement(prev))
      replace(p, std::move_if_noexcept(*prev));
    destroy_item(m_first);
    increment(m_first);
    --m_size;
    return p == pos.m_it ? begin() : pos;
  }

  //! Erase the range <code>[first, last)</code>.
  iterator rerase(iterator first, iterator last) {
    // check for uninitialized or invalidated iterator
    assert(first.is_valid(this));
    // check for uninitialized or invalidated iterator
    assert(last.is_valid(this));
    assert(first <= last); // check for wrong range
    if (first == last)
      return first;
    pointer p = map_pointer(last.m_it);
    last.m_it = p;
    while (first.m_it != m_first) {
      decrement(first.m_it);
      decrement(p);
      replace(p, std::move_if_noexcept(*first.m_it));
    }
    do {
      destroy_item(m_first);
      increment(m_first);
      --m_size;
    } while (m_first != p);
    if (m_first == last.m_it)
      return begin();
    decrement(last.m_it);
    return iterator(this, last.m_it);
  }

  void erase_begin(size_type n) {
    assert(n <= size()); // check for n greater than size
    erase_begin(n, std::is_scalar<value_type>());
  }

  void erase_end(size_type n) {
    assert(n <= size()); // check for n greater than size
    erase_end(n, std::is_scalar<value_type>());
  }

  void clear() noexcept {
    destroy_content();
    m_size = 0;
  }

private:
  // Helper methods

  //! Check if the <code>index</code> is valid.
  void check_position(size_type index) const {
    if (index >= size())
      throw std::out_of_range("circular_buffer");
  }

  //! Increment the pointer.
  template <class Pointer> void increment(Pointer &p) const {
    if (++p == m_end)
      p = m_buff;
  }

  //! Decrement the pointer.
  template <class Pointer> void decrement(Pointer &p) const {
    if (p == m_buff)
      p = m_end;
    --p;
  }

  //! Add <code>n</code> to the pointer.
  template <class Pointer> Pointer add(Pointer p, difference_type n) const {
    return p + (n < (m_end - p) ? n : n - capacity());
  }

  //! Subtract <code>n</code> from the pointer.
  template <class Pointer> Pointer sub(Pointer p, difference_type n) const {
    return p - (n > (p - m_buff) ? n - capacity() : n);
  }

  //! Map the null pointer to virtual end of circular buffer.
  pointer map_pointer(pointer p) const { return p == 0 ? m_last : p; }

  //! Allocate memory.
  pointer allocate(size_type n) {
    if (n > max_size())
      throw std::length_error("circular_buffer");
    return (n == 0) ? 0 : m_alloc.allocate(n);
  }

  //! Deallocate memory.
  void deallocate(pointer p, size_type n) {
    if (p != 0)
      m_alloc.deallocate(p, n);
  }

  //! Does the pointer point to the uninitialized memory?
  bool is_uninitialized(const_pointer p) const noexcept {
    return p >= m_last && (m_first < m_last || p < m_first);
  }

  //! Replace an element.
  void replace(pointer pos, param_value_type item) {
    *pos = item;
#if CB_ENABLE_DEBUG
    invalidate_iterators(iterator(this, pos));
#endif
  }

  //! Replace an element.
  void replace(pointer pos, rvalue_type item) {
    *pos = std::move(item);
#if CB_ENABLE_DEBUG
    invalidate_iterators(iterator(this, pos));
#endif
  }

  //! Construct or replace an element.
  /*!
      <code>construct</code> has to be set to <code>true</code> if and only if
      <code>pos</code> points to an uninitialized memory.
  */
  void construct_or_replace(bool construct, pointer pos,
                            param_value_type item) {
    if (construct)
      std::allocator_traits<Alloc>::construct(m_alloc, std::addressof(*pos),
                                              item);
    else
      replace(pos, item);
  }

  //! Construct or replace an element.
  /*!
      <code>construct</code> has to be set to <code>true</code> if and only if
      <code>pos</code> points to an uninitialized memory.
  */
  void construct_or_replace(bool construct, pointer pos, rvalue_type item) {
    if (construct)
      std::allocator_traits<Alloc>::construct(m_alloc, std::addressof(*pos),
                                              std::move(item));
    else
      replace(pos, std::move(item));
  }

  //! Destroy an item.
  void destroy_item(pointer p) {
    std::allocator_traits<Alloc>::destroy(m_alloc, std::addressof(*p));
#if CB_ENABLE_DEBUG
    invalidate_iterators(iterator(this, p));
    cb_details::do_fill_uninitialized_memory(p, sizeof(value_type));
#endif
  }

  //! Destroy an item only if it has been constructed.
  void destroy_if_constructed(pointer pos) {
    if (is_uninitialized(pos))
      destroy_item(pos);
  }

  //! Destroy the whole content of the circular buffer.
  void destroy_content() {
#if CB_ENABLE_DEBUG
    destroy_content(std::false_type());
#else
    destroy_content(std::is_scalar<value_type>());
#endif
  }

  //! Specialized destroy_content method.
  void destroy_content(const std::true_type &) {
    m_first = add(m_first, size());
  }

  //! Specialized destroy_content method.
  void destroy_content(const std::false_type &) {
    for (size_type ii = 0; ii < size(); ++ii, increment(m_first))
      destroy_item(m_first);
  }

  //! Destroy content and free allocated memory.
  void destroy() noexcept {
    destroy_content();
    deallocate(m_buff, capacity());
#if CB_ENABLE_DEBUG
    m_buff = 0;
    m_first = 0;
    m_last = 0;
    m_end = 0;
#endif
  }

  //! Initialize the internal buffer.
  void initialize_buffer(capacity_type buffer_capacity) {
    m_buff = allocate(buffer_capacity);
    m_end = m_buff + buffer_capacity;
  }

  //! Initialize the internal buffer.
  void initialize_buffer(capacity_type buffer_capacity, param_value_type item) {
    initialize_buffer(buffer_capacity);
    {
      try {
        cb_details::uninitialized_fill_n_with_alloc(m_buff, size(), item,
                                                    m_alloc);
      } catch (...) {
        deallocate(m_buff, size());
        throw;
      }
    }
  }

  //! Specialized initialize method.
  template <class IntegralType>
  void initialize(IntegralType n, IntegralType item, const std::true_type &) {
    m_size = static_cast<size_type>(n);
    initialize_buffer(size(), item);
    m_first = m_last = m_buff;
  }

  //! Specialized initialize method.
  template <class Iterator>
  void initialize(Iterator first, Iterator last, const std::false_type &) {
    static_assert((std::is_convertible<
                      typename std::iterator_traits<Iterator>::value_type,
                      value_type>::value),
                  "Invalid iterator type");
    initialize(
        first, last,
        typename std::iterator_traits<Iterator>::iterator_category::type());
  }

  //! Specialized initialize method.
  template <class InputIterator>
  void initialize(InputIterator first, InputIterator last,
                  const std::input_iterator_tag &) {
    std::deque<value_type, allocator_type> tmp(first, last, m_alloc);
    size_type distance = tmp.size();
    initialize(distance, std::make_move_iterator(tmp.begin()),
               std::make_move_iterator(tmp.end()), distance);
  }

  //! Specialized initialize method.
  template <class ForwardIterator>
  void initialize(ForwardIterator first, ForwardIterator last,
                  const std::forward_iterator_tag &) {
    assert(std::distance(first, last) >= 0); // check for wrong range
    size_type distance = std::distance(first, last);
    initialize(distance, first, last, distance);
  }

  //! Specialized initialize method.
  template <class IntegralType>
  void initialize(capacity_type buffer_capacity, IntegralType n,
                  IntegralType item, const std::true_type &) {
    assert(buffer_capacity >=
           static_cast<size_type>(n)); // check for capacity lower than n
    m_size = static_cast<size_type>(n);
    initialize_buffer(buffer_capacity, item);
    m_first = m_buff;
    m_last = buffer_capacity == size() ? m_buff : m_buff + size();
  }

  //! Specialized initialize method.
  template <class Iterator>
  void initialize(capacity_type buffer_capacity, Iterator first, Iterator last,
                  const std::false_type &) {
    static_assert((std::is_convertible<
                      typename std::iterator_traits<Iterator>::value_type,
                      value_type>::value),
                  "Invalid iterator type");
    initialize(
        buffer_capacity, first, last,
        typename std::iterator_traits<Iterator>::iterator_category::type());
  }

  //! Specialized initialize method.
  template <class InputIterator>
  void initialize(capacity_type buffer_capacity, InputIterator first,
                  InputIterator last, const std::input_iterator_tag &) {
    initialize_buffer(buffer_capacity);
    m_first = m_last = m_buff;
    m_size = 0;
    if (buffer_capacity == 0)
      return;
    while (first != last && !full()) {
      std::allocator_traits<Alloc>::construct(m_alloc, std::addressof(*m_last),
                                              *first++);
      increment(m_last);
      ++m_size;
    }
    while (first != last) {
      replace(m_last, *first++);
      increment(m_last);
      m_first = m_last;
    }
  }

  //! Specialized initialize method.
  template <class ForwardIterator>
  void initialize(capacity_type buffer_capacity, ForwardIterator first,
                  ForwardIterator last, const std::forward_iterator_tag &) {
    assert(std::distance(first, last) >= 0); // check for wrong range
    initialize(buffer_capacity, first, last, std::distance(first, last));
  }

  //! Initialize the circular buffer.
  template <class ForwardIterator>
  void initialize(capacity_type buffer_capacity, ForwardIterator first,
                  ForwardIterator last, size_type distance) {
    initialize_buffer(buffer_capacity);
    m_first = m_buff;
    if (distance > buffer_capacity) {
      std::advance(first, distance - buffer_capacity);
      m_size = buffer_capacity;
    } else {
      m_size = distance;
    }
    {
      try {
        m_last = cb_details::uninitialized_copy(first, last, m_buff, m_alloc);
      } catch (...) {
        deallocate(m_buff, buffer_capacity);
        throw;
      }
    }
    if (m_last == m_end)
      m_last = m_buff;
  }

  //! Reset the circular buffer.
  void reset(pointer buff, pointer last, capacity_type new_capacity) {
    destroy();
    m_size = last - buff;
    m_first = m_buff = buff;
    m_end = m_buff + new_capacity;
    m_last = last == m_end ? m_buff : last;
  }

  //! Specialized method for swapping the allocator.
  void swap_allocator(circular_buffer<T, Alloc> &, const std::true_type &) {
    // Swap is not needed because allocators have no state.
  }

  //! Specialized method for swapping the allocator.
  void swap_allocator(circular_buffer<T, Alloc> &cb, const std::false_type &) {
    adl_move_swap(m_alloc, cb.m_alloc);
  }

  //! Specialized assign method.
  template <class IntegralType>
  void assign(IntegralType n, IntegralType item, const std::true_type &) {
    assign(static_cast<size_type>(n), static_cast<value_type>(item));
  }

  //! Specialized assign method.
  template <class Iterator>
  void assign(Iterator first, Iterator last, const std::false_type &) {
    static_assert((std::is_convertible<
                      typename std::iterator_traits<Iterator>::value_type,
                      value_type>::value),
                  "Invalid iterator type");
    assign(first, last,
           typename std::iterator_traits<Iterator>::iterator_category::type());
  }

  //! Specialized assign method.
  template <class InputIterator>
  void assign(InputIterator first, InputIterator last,
              const std::input_iterator_tag &) {
    std::deque<value_type, allocator_type> tmp(first, last, m_alloc);
    size_type distance = tmp.size();
    assign_n(distance, distance,
             cb_details::make_assign_range(std::make_move_iterator(tmp.begin()),
                                           std::make_move_iterator(tmp.end()),
                                           m_alloc));
  }

  //! Specialized assign method.
  template <class ForwardIterator>
  void assign(ForwardIterator first, ForwardIterator last,
              const std::forward_iterator_tag &) {
    assert(std::distance(first, last) >= 0); // check for wrong range
    size_type distance = std::distance(first, last);
    assign_n(distance, distance,
             cb_details::make_assign_range(first, last, m_alloc));
  }

  //! Specialized assign method.
  template <class IntegralType>
  void assign(capacity_type new_capacity, IntegralType n, IntegralType item,
              const std::true_type &) {
    assign(new_capacity, static_cast<size_type>(n),
           static_cast<value_type>(item));
  }

  //! Specialized assign method.
  template <class Iterator>
  void assign(capacity_type new_capacity, Iterator first, Iterator last,
              const std::false_type &) {
    static_assert((std::is_convertible<
                      typename std::iterator_traits<Iterator>::value_type,
                      value_type>::value),
                  "Invalid iterator type");
    assign(new_capacity, first, last,
           typename std::iterator_traits<Iterator>::iterator_category::type());
  }

  //! Specialized assign method.
  template <class InputIterator>
  void assign(capacity_type new_capacity, InputIterator first,
              InputIterator last, const std::input_iterator_tag &) {
    if (new_capacity == capacity()) {
      clear();
      insert(begin(), first, last);
    } else {
      circular_buffer<value_type, allocator_type> tmp(new_capacity, first, last,
                                                      m_alloc);
      tmp.swap(*this);
    }
  }

  //! Specialized assign method.
  template <class ForwardIterator>
  void assign(capacity_type new_capacity, ForwardIterator first,
              ForwardIterator last, const std::forward_iterator_tag &) {
    assert(std::distance(first, last) >= 0); // check for wrong range
    size_type distance = std::distance(first, last);
    if (distance > new_capacity) {
      std::advance(first, distance - new_capacity);
      distance = new_capacity;
    }
    assign_n(new_capacity, distance,
             cb_details::make_assign_range(first, last, m_alloc));
  }

  //! Helper assign method.
  template <class Functor>
  void assign_n(capacity_type new_capacity, size_type n, const Functor &fnc) {
    if (new_capacity == capacity()) {
      destroy_content();
      {
        try {
          fnc(m_buff);
        } catch (...) {
          m_size = 0;
          throw;
        }
      }
    } else {
      pointer buff = allocate(new_capacity);
      {
        try {
          fnc(buff);
        } catch (...) {
          deallocate(buff, new_capacity);
          throw;
        }
      }
      destroy();
      m_buff = buff;
      m_end = m_buff + new_capacity;
    }
    m_size = n;
    m_first = m_buff;
    m_last = add(m_buff, size());
  }

  //! Helper insert method.
  template <class ValT> iterator insert_item(const iterator &pos, ValT item) {
    pointer p = pos.m_it;
    if (p == 0) {
      construct_or_replace(!full(), m_last, static_cast<ValT>(item));
      p = m_last;
    } else {
      pointer src = m_last;
      pointer dest = m_last;
      bool construct = !full();
      {
        try {
          while (src != p) {
            decrement(src);
            construct_or_replace(construct, dest, std::move_if_noexcept(*src));
            decrement(dest);
            construct = false;
          }
          replace(p, static_cast<ValT>(item));
        } catch (...) {
          if (!construct && !full()) {
            increment(m_last);
            ++m_size;
          }
          throw;
        }
      }
    }
    increment(m_last);
    if (full())
      m_first = m_last;
    else
      ++m_size;
    return iterator(this, p);
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
    static_assert((std::is_convertible<
                      typename std::iterator_traits<Iterator>::value_type,
                      value_type>::value),
                  "Invalid iterator type");
    insert(pos, first, last,
           typename std::iterator_traits<Iterator>::iterator_category::type());
  }

  //! Specialized insert method.
  template <class InputIterator>
  void insert(iterator pos, InputIterator first, InputIterator last,
              const std::input_iterator_tag &) {
    if (!full() || pos != begin()) {
      for (; first != last; ++pos)
        pos = insert(pos, *first++);
    }
  }

  //! Specialized insert method.
  template <class ForwardIterator>
  void insert(const iterator &pos, ForwardIterator first, ForwardIterator last,
              const std::forward_iterator_tag &) {
    assert(std::distance(first, last) >= 0); // check for wrong range
    size_type n = std::distance(first, last);
    if (n == 0)
      return;
    size_type copy = capacity() - (end() - pos);
    if (copy == 0)
      return;
    if (n > copy) {
      std::advance(first, n - copy);
      n = copy;
    }
    insert_n(pos, n, cb_details::iterator_wrapper<ForwardIterator>(first));
  }

  //! Helper insert method.
  template <class Wrapper>
  void insert_n(const iterator &pos, size_type n, const Wrapper &wrapper) {
    size_type construct = reserve();
    if (construct > n)
      construct = n;
    if (pos.m_it == 0) {
      size_type ii = 0;
      pointer p = m_last;
      {
        try {
          for (; ii < construct; ++ii, increment(p))
            std::allocator_traits<Alloc>::construct(m_alloc, std::addressof(*p),
                                                    *wrapper());
          for (; ii < n; ++ii, increment(p))
            replace(p, *wrapper());
        } catch (...) {
          size_type constructed = (std::min)(ii, construct);
          m_last = add(m_last, constructed);
          m_size += constructed;
          throw;
        }
      }
    } else {
      pointer src = m_last;
      pointer dest = add(m_last, n - 1);
      pointer p = pos.m_it;
      size_type ii = 0;
      {
        try {
          while (src != pos.m_it) {
            decrement(src);
            construct_or_replace(is_uninitialized(dest), dest, *src);
            decrement(dest);
          }
          for (; ii < n; ++ii, increment(p))
            construct_or_replace(is_uninitialized(p), p, *wrapper());
        } catch (...) {
          for (p = add(m_last, n - 1); p != dest; decrement(p))
            destroy_if_constructed(p);
          for (n = 0, p = pos.m_it; n < ii; ++n, increment(p))
            destroy_if_constructed(p);
          throw;
        }
      }
    }
    m_last = add(m_last, n);
    m_first = add(m_first, n - construct);
    m_size += construct;
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
    static_assert((std::is_convertible<
                      typename std::iterator_traits<Iterator>::value_type,
                      value_type>::value),
                  "Invalid iterator type");
    rinsert(pos, first, last,
            typename std::iterator_traits<Iterator>::iterator_category::type());
  }

  //! Specialized insert method.
  template <class InputIterator>
  void rinsert(iterator pos, InputIterator first, InputIterator last,
               const std::input_iterator_tag &) {
    if (!full() || pos.m_it != 0) {
      for (; first != last; ++pos) {
        pos = rinsert(pos, *first++);
        if (pos.m_it == 0)
          break;
      }
    }
  }

  //! Specialized rinsert method.
  template <class ForwardIterator>
  void rinsert(const iterator &pos, ForwardIterator first, ForwardIterator last,
               const std::forward_iterator_tag &) {
    assert(std::distance(first, last) >= 0); // check for wrong range
    rinsert_n(pos, std::distance(first, last),
              cb_details::iterator_wrapper<ForwardIterator>(first));
  }

  //! Helper rinsert method.
  template <class Wrapper>
  void rinsert_n(const iterator &pos, size_type n, const Wrapper &wrapper) {
    if (n == 0)
      return;
    iterator b = begin();
    size_type copy = capacity() - (pos - b);
    if (copy == 0)
      return;
    if (n > copy)
      n = copy;
    size_type construct = reserve();
    if (construct > n)
      construct = n;
    if (pos == b) {
      pointer p = sub(m_first, n);
      size_type ii = n;
      {
        try {
          for (; ii > construct; --ii, increment(p))
            replace(p, *wrapper());
          for (; ii > 0; --ii, increment(p))
            std::allocator_traits<Alloc>::construct(m_alloc, std::addressof(*p),
                                                    *wrapper());
        } catch (...) {
          size_type constructed = ii < construct ? construct - ii : 0;
          m_last = add(m_last, constructed);
          m_size += constructed;
          throw;
        }
      }
    } else {
      pointer src = m_first;
      pointer dest = sub(m_first, n);
      pointer p = map_pointer(pos.m_it);
      {
        try {
          while (src != p) {
            construct_or_replace(is_uninitialized(dest), dest, *src);
            increment(src);
            increment(dest);
          }
          for (size_type ii = 0; ii < n; ++ii, increment(dest))
            construct_or_replace(is_uninitialized(dest), dest, *wrapper());
        } catch (...) {
          for (src = sub(m_first, n); src != dest; increment(src))
            destroy_if_constructed(src);
          throw;
        }
      }
    }
    m_first = sub(m_first, n);
    m_last = sub(m_last, n - construct);
    m_size += construct;
  }

  //! Specialized erase_begin method.
  void erase_begin(size_type n, const std::true_type &) {
    m_first = add(m_first, n);
    m_size -= n;
  }

  //! Specialized erase_begin method.
  void erase_begin(size_type n, const std::false_type &) {
    iterator b = begin();
    rerase(b, b + n);
  }

  //! Specialized erase_end method.
  void erase_end(size_type n, const std::true_type &) {
    m_last = sub(m_last, n);
    m_size -= n;
  }

  //! Specialized erase_end method.
  void erase_end(size_type n, const std::false_type &) {
    iterator e = end();
    erase(e - n, e);
  }
};

template <class T, class Alloc>
inline bool operator==(const circular_buffer<T, Alloc> &lhs,
                       const circular_buffer<T, Alloc> &rhs) {
  return lhs.size() == rhs.size() &&
         std::equal(lhs.begin(), lhs.end(), rhs.begin());
}
template <class T, class Alloc>
inline bool operator<(const circular_buffer<T, Alloc> &lhs,
                      const circular_buffer<T, Alloc> &rhs) {
  return std::lexicographical_compare(lhs.begin(), lhs.end(), rhs.begin(),
                                      rhs.end());
}

template <class T, class Alloc>
inline bool operator!=(const circular_buffer<T, Alloc> &lhs,
                       const circular_buffer<T, Alloc> &rhs) {
  return !(lhs == rhs);
}

template <class T, class Alloc>
inline bool operator>(const circular_buffer<T, Alloc> &lhs,
                      const circular_buffer<T, Alloc> &rhs) {
  return rhs < lhs;
}

template <class T, class Alloc>
inline bool operator<=(const circular_buffer<T, Alloc> &lhs,
                       const circular_buffer<T, Alloc> &rhs) {
  return !(rhs < lhs);
}

template <class T, class Alloc>
inline bool operator>=(const circular_buffer<T, Alloc> &lhs,
                       const circular_buffer<T, Alloc> &rhs) {
  return !(lhs < rhs);
}

template <class T, class Alloc>
inline void swap(circular_buffer<T, Alloc> &lhs,
                 circular_buffer<T, Alloc> &rhs) noexcept {
  lhs.swap(rhs);
}

} // namespace cb
