//
//=======================================================================
// Copyright 2015
// Author: Alex Hagen-Zanker
// University of Surrey
//
// Distributed under the MIT Licence (http://opensource.org/licenses/MIT)
//=======================================================================
//
// raster_traits are used to inspect properties of rasters
//     - value_type
//     - coordinate_type
//     - index_type
//     - iterator
// raster_operations define free functions to be overloaded for each raster type. 
// that default to appropriate member functions. 

#ifndef RASTER_TRAITS_H_AHZ
#define RASTER_TRAITS_H_AHZ

namespace raster_util {

  namespace raster_traits
  {
    template<typename Raster>
    struct value_type
    {
      typedef typename Raster::value_type type;
    };

    template<typename Raster>
    struct coordinate_type
    {
      typedef typename Raster::coordinate_type type;
    };

    template<typename Raster>
    struct index_type
    {
      typedef typename Raster::index_type type;
    };

    template <typename OrientationTag, typename ElementTag, typename AccessTag, typename RasterType>
    struct iterator
    {
      struct has_no_default_implementation{};
      typedef has_no_default_implementation type;
    };

  };

  namespace raster_operations
  {
    template <typename OrientationTag, typename ElementTag, typename AccessTag, typename Raster>
    typename raster_traits::iterator< OrientationTag, ElementTag, AccessTag, Raster>::type
      begin(Raster& r) 
    {
        typedef typename raster_traits::iterator< OrientationTag, ElementTag, AccessTag, Raster>::type
          iterator;
        return r.template begin<iterator>();
    }
    template <typename OrientationTag, typename ElementTag, typename AccessTag, typename Raster>
    typename raster_traits::iterator< OrientationTag, ElementTag, AccessTag, Raster>::type
      end(Raster& r)
    {
        typedef typename raster_traits::iterator< OrientationTag, ElementTag, AccessTag, Raster>::type
          iterator;
        return r.template end<iterator>();
    }
  }


  namespace raster_operations
  {
    template<typename Raster>
    typename raster_traits::index_type<Raster>::type size1(const Raster& r)
    {
      return r.size1();
    }

    template<typename Raster>
    typename raster_traits::index_type<Raster>::type  size2(const Raster& r)
    {
      return r.size2();
    }
  }

  namespace raster_operations
  {
    template<typename Raster>
    typename raster_traits::index_type<Raster>::type block_size1(const Raster& r)
    {
      return r.block_size1();
    }

    template<typename Raster>
    typename raster_traits::index_type<Raster>::type  block_size2(const Raster& r)
    {
      return r.block_size2();
    }

    template<typename Raster>
    typename raster_traits::value_type<Raster>::type get_pixel_in_block(
      const Raster& r,
      typename raster_traits::index_type<Raster>::type block,
      typename raster_traits::index_type<Raster>::type pixel_in_block)
    {
      return r.get_pixel_in_block(block, pixel_in_block);
    }

    template<typename Raster>
    void put_pixel_in_block(
      Raster& r,
      const typename raster_traits::value_type<Raster>::type& value,
      typename raster_traits::index_type<Raster>::type block,
      typename raster_traits::index_type<Raster>::type pixel_in_block)
    {
      r.put_pixel_in_block(block, pixel_in_block, value);
    }
  };

  template <typename OrientationTag, typename ElementTag, typename AccessTag, typename RasterType>
  struct raster_view
  {
    typedef typename raster_traits::iterator<
      OrientationTag,
      ElementTag,
      AccessTag,
      RasterType>::type iterator;

    raster_view(RasterType* raster) : m_raster(raster)
    {

    }
    iterator begin()
    {
      return raster_operations::begin<OrientationTag, ElementTag, AccessTag, RasterType>(*m_raster);
    }

    iterator end()
    {
      return raster_operations::end<OrientationTag, ElementTag, AccessTag, RasterType>(*m_raster);
    }
    /* should depend on type of elem
    int size1() const
    {
    return raster_operations::size1(m_raster);
    }

    int size2() const
    {
    return raster_operations::size2(m_raster);
    }
    */

    RasterType* m_raster;
  };


} // namespace raster_util 

#endif