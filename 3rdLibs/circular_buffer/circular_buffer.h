/*
 * @Author: chenjingyu
 * @Date: 2023-11-24 15:55:57
 * @LastEditTime: 2023-11-24 15:57:09
 * @Description: Circular buffer
 * @FilePath: \CodeBlocks\C++\3rdLibs\circularbuffer\circular_buffer.h
 */
#pragma once

#include <memory>
#include <type_traits>
#include <cassert>
#include <iterator>

#include "details.h"
#include "base.h"
#include "space_optimized.h"

namespace cb {
template<class T, class Alloc = std::allocator<T> >
class circular_buffer;

template<class T, class Alloc = std::allocator<T> >
class circular_buffer_space_optimized;

} // namespace cb