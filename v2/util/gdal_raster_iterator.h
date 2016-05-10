//
//=======================================================================
// Copyright 2015
// Author: Alex Hagen-Zanker
// University of Surrey
//
// Distributed under the MIT Licence (http://opensource.org/licenses/MIT)
//=======================================================================
//
// This file presents iterators for gdal_raster, but has been generalized to 
// work with any raster for which the raster_operation and raster_traits have 
// been defined see also raster_traits.h
//
// The iterators (try to) use the concepts as proposed in the context of the 
// BOOST library
//
// http://www.boost.org/doc/libs/1_52_0/libs/iterator/doc/new-iter-concepts.html
//
// keep in mind that these are not the concepts of the standard, because the 
// dereference does not return a reference to the elements, but a proxy
//
// gdal_iterator<Raster> and gdal_trans_iterator are random access iterators for 
// Raster 
// gdal_iterator visits all cells row-by-row and each row col-by-col
// gdal_trans_iterator visits all cells col-by-col and each col row-by-row
// The iterators implement the RandomAccessIterator concept, except that 
// dereferencing the iterator yields a dereference_proxy instead of a reference
// to the element.
// The proxy that can be used in much the same way: 
// like this:         *iter = value; 
// or like this:      value_type value = *iter;
// but not like this: value_type& value  = *iter
//
// Raster must implement the BlockedRaster concept, of which gdal_raster<T> is 
// the prime model.
//
// TODO: the way in which const_iterators are defined is maybe not up to snuff

#ifndef GDAL_RASTER_ITERATOR_H_AHZ
#define GDAL_RASTER_ITERATOR_H_AHZ

#include "dereference_proxy.h"
#include "raster_traits.h"
#include "raster_iterator.h"

#include <boost/iterator/iterator_categories.hpp>
#include <boost/iterator/iterator_facade.hpp>

#include <type_traits>

namespace raster_util {

  template<typename Raster>
  class gdal_iterator
    : public boost::iterator_facade<
    gdal_iterator<Raster>,
    typename raster_traits::value_type<Raster>::type,
    boost::random_access_traversal_tag,
    dereference_proxy<gdal_iterator<Raster>,
    typename raster_traits::value_type<Raster>::type>,
    std::ptrdiff_t>
  {

    typedef gdal_iterator<Raster> this_type;
    typedef gdal_iterator<const Raster> this_type_const;
    typedef gdal_iterator<typename boost::remove_const<Raster>::type>
      this_type_non_const;
    typedef typename raster_traits::value_type<Raster>::type raster_value_type;
    typedef dereference_proxy<this_type, raster_value_type> dereference_proxy_td;
    // typedef dereference_proxy reference;

  public:
    typedef raster_value_type value_type;
    typedef typename raster_traits::coordinate_type<Raster>::type coordinate_type;
    typedef typename raster_traits::index_type<Raster>::type index_type;
    typedef dereference_proxy_td reference;

    gdal_iterator() : m_raster(NULL)
    {}

    gdal_iterator(Raster* r) : m_raster(r)
    {
      find_begin();
    }

    void find(const coordinate_type& coord)
    {
      m_coordinates = coord;
      change_block();
    }

    void find_begin()
    {
      m_coordinates.row = 0;
      m_coordinates.col = 0;
      change_block();
    }

    void find_end()
    {
      m_coordinates.row = size1();
      m_coordinates.col = 0;
      change_block();
    }

    const coordinate_type& get_coordinates() const
    {
      return m_coordinates;
    }

    value_type get() const
    {
      return	raster_operations::get_pixel_in_block(*m_raster, m_major_index,
        m_minor_index);
    }

    void put(const value_type& value) const
    {
      raster_operations::put_pixel_in_block(*m_raster, value, m_major_index,
        m_minor_index);
    }

  private:

    friend class boost::iterator_core_access;

    void decrement()
    {
      if (m_coordinates.col != m_stretch_begin_col) {
        --m_coordinates.col;
        --m_minor_index;
      }
      else {
        if (m_coordinates.col == 0) {
          m_coordinates.col = size2() - 1;
          --m_coordinates.row;
        }
        else {
          --m_coordinates.col;
        }
        change_block();
      }
    }

    void increment()
    {
      if (++m_coordinates.col != m_stretch_end_col) {
        ++m_minor_index;
      }
      else {
        if (m_coordinates.col == size2()) {
          m_coordinates.col = 0;
          ++m_coordinates.row;
        }
        change_block();
      }
    }

    void advance(std::ptrdiff_t n)
    {
      std::ptrdiff_t new_index = index() + n;
      m_coordinates.row = new_index / size2();
      m_coordinates.col = new_index % size2();
      change_block();
    }

    void change_block()
    {
      const index_type block_row_size = raster_operations::block_size1(*m_raster);
      const index_type block_col_size = raster_operations::block_size2(*m_raster);

      const index_type minor_row = m_coordinates.row % block_row_size;
      const index_type minor_col = m_coordinates.col % block_col_size;

      const index_type major_row = m_coordinates.row / block_row_size;
      const index_type major_col = m_coordinates.col / block_col_size;

      const index_type major_cols_in_row = 1 + (size2() - 1) / block_col_size;

      m_minor_index = minor_row * block_col_size + minor_col;
      m_major_index = major_row * major_cols_in_row + major_col;

      m_stretch_begin_col = major_col * block_col_size;
      m_stretch_end_col = std::min(m_stretch_begin_col + block_col_size, size2());
    }

    bool equal(const this_type_non_const& other) const
    {
      return this->m_coordinates == other.m_coordinates;
    }

    bool equal(const this_type_const& other) const
    {
      return this->m_coordinates == other.m_coordinates;
    }

    dereference_proxy_td dereference() const
    {
      return dereference_proxy_td(this);
    }

    std::ptrdiff_t distance_to(const this_type_non_const& other) const
    {
      return other.index() - this->index();
    }

    std::ptrdiff_t distance_to(const this_type_const& other) const
    {
      return other.index() - this->index();
    }

    std::ptrdiff_t index() const
    {
      return m_coordinates.row * size2() + m_coordinates.col;
    }

    index_type size1() const
    {
      return raster_operations::size1(*m_raster);
    }

    index_type size2() const
    {
      return raster_operations::size2(*m_raster);
    }

    coordinate_type m_coordinates;

    index_type m_stretch_begin_col;
    index_type m_stretch_end_col;

    index_type m_minor_index;
    index_type m_major_index;

    Raster* m_raster;
  };

  template<typename Raster>
  class gdal_trans_iterator
    : public boost::iterator_facade<
    gdal_trans_iterator<Raster>,
    typename raster_traits::value_type<Raster>::type,
    boost::random_access_traversal_tag,
    dereference_proxy<gdal_trans_iterator<Raster>,
    typename raster_traits::value_type<Raster>::type >,
    std::ptrdiff_t>
  {
    typedef gdal_trans_iterator<Raster> this_type;
    typedef gdal_trans_iterator<const Raster> this_type_const;
    typedef gdal_trans_iterator<typename std::remove_const<Raster>::type>
      this_type_non_const;

  public:
    typedef typename Raster::coordinate_type coordinate_type;
    typedef typename Raster::index_type index_type;
    typedef typename raster_traits::value_type<Raster>::type value_type;
    typedef dereference_proxy<this_type, value_type> dereference_proxy_td;

    gdal_trans_iterator() : m_raster(NULL)
    {}

    gdal_trans_iterator(Raster* r) : m_raster(r)
    {
      find_begin();
    }

    void find(const coordinate_type& coord)
    {
      m_coordinates = coord;
      change_block();
    }

    void find_begin()
    {
      m_coordinates.row = 0;
      m_coordinates.col = 0;
      change_block();
    }

    void find_end()
    {
      m_coordinates.row = 0;
      m_coordinates.col = size2();
      change_block();
    }

    const coordinate_type& get_coordinates() const
    {
      return m_coordinates;
    }

    value_type get() const
    {
      return	raster_operations::get_pixel_in_block(*m_raster, m_major_index,
        m_minor_index);
    }

    void put(const value_type& value) const
    {
      return raster_operations::put_pixel_in_block(*m_raster, value,
        m_major_index, m_minor_index);
    }

  private:

    friend class boost::iterator_core_access;
    friend struct dereference_proxy<this_type, value_type>;


    void decrement()
    {
      if (m_coordinates.row != m_stretch_begin_row) {
        --m_coordinates.row;
        m_minor_index -= raster_operations::block_size2(*m_raster);
      }
      else {
        if (m_coordinates.row == 0) {
          m_coordinates.row = size1() - 1;
          --m_coordinates.col;
        }
        else {
          --m_coordinates.col;
        }
        change_block();
      }
    }

    void increment()
    {
      if (++m_coordinates.row != m_stretch_end_row) {
        m_minor_index += raster_operations::block_size2(*m_raster);
      }
      else {
        if (m_coordinates.row == size1()) {
          m_coordinates.row = 0;
          ++m_coordinates.col;
        }
        change_block();
      }
    }

    void advance(std::ptrdiff_t n)
    {
      std::ptrdiff_t new_index = index() + n;
      m_coordinates.row = new_index % size1();
      m_coordinates.col = new_index / size1();
      change_block();
    }

    void change_block()
    {
      const index_type block_row_size = raster_operations::block_size1(*m_raster);
      const index_type block_col_size = raster_operations::block_size2(*m_raster);

      const index_type minor_row = m_coordinates.row % block_row_size;
      const index_type minor_col = m_coordinates.col % block_col_size;

      const index_type major_row = m_coordinates.row / block_row_size;
      const index_type major_col = m_coordinates.col / block_col_size;

      const index_type major_cols_in_row = 1 + (size2() - 1) / block_col_size;

      m_minor_index = minor_row * block_col_size + minor_col;
      m_major_index = major_row * major_cols_in_row + major_col;

      m_stretch_begin_row = major_row * block_row_size;
      m_stretch_end_row = std::min(m_stretch_begin_row + block_row_size, size1());
    }

    bool equal(const this_type_non_const& other) const
    {
      return this->m_coordinates == other.m_coordinates;
    }

    bool equal(const this_type_const& other) const
    {
      return this->m_coordinates == other.m_coordinates;
    }

    dereference_proxy_td dereference() const
    {
      return dereference_proxy_td(this);
    }

    std::ptrdiff_t distance_to(const this_type_non_const& other) const
    {
      return other.index() - this->index();
    }

    std::ptrdiff_t distance_to(const this_type_const& other) const
    {
      return other.index() - this->index();
    }

    std::ptrdiff_t index() const
    {
      return m_coordinates.col * size1() + m_coordinates.row;
    }

    index_type size1() const
    {
      return raster_operations::size1(*m_raster);
    }

    index_type size2() const
    {
      return raster_operations::size2(*m_raster);
    }

    coordinate_type m_coordinates;

    index_type m_stretch_begin_row;
    index_type m_stretch_end_row;

    index_type m_major_index;
    index_type m_minor_index;

    Raster* m_raster;
  };

} //namespace raster_util 
#endif

