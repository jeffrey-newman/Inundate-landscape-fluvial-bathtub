//
//=======================================================================
// Copyright 2015
// Author: Alex Hagen-Zanker
// University of Surrey
//
// Distributed under the MIT Licence (http://opensource.org/licenses/MIT)
//=======================================================================
//
// Implements a zip range, it is not a fully functional generic zip range, 
// but supports the forward-only iteration necessary in the project

#ifndef ZIP_RANGE_H_AHZ
#define ZIP_RANGE_H_AHZ

#include <functional>
#include <tuple>
#include <utility>

namespace raster_util {
  template<typename ...Ts> struct pack{};
  template<typename T> struct unwrap { typedef T type; };
  template<typename T> struct unwrap<std::reference_wrapper<T>> { typedef T& type; };
  template<typename T> struct unwrap_unref { typedef T type; }; 
  template<typename T> struct unwrap_unref<std::reference_wrapper<T> > { typedef T type; };
 
  template<typename T> struct full_decay
  {
    typedef typename std::decay<typename unwrap<T>::type>::type type;
  };

  template<int ...> struct seq { };

  template<int N, int ...S> struct gens : gens<N - 1, N - 1, S...> { };
  template<int ...S> struct gens<0, S...> { typedef seq<S...> type; };

  struct empty {};

  template<typename...Args> void ignore(Args...)
  {}

  template < typename... Iterators >
  class zip_iterator
  {
    typedef std::tuple<typename unwrap<Iterators>::type...> unwrapped_iterators;
    typedef std::tuple<typename unwrap_unref<Iterators>::type...> unreffed_iterators;

  public:
    zip_iterator(const zip_iterator& it) : iterators(it.iterators)
    { }

    template<typename... InIterators>
    explicit zip_iterator(InIterators&&...its) : iterators(std::forward<InIterators>(its)...)
    { }

    zip_iterator& operator=(const zip_iterator& it)
    {
      iterators = it.iterators;
      return *this;
    }

 private:
    typedef typename gens<sizeof...(Iterators)>::type tuple_indices;

    // Workaround for the Visual Studio bug
    // http://stackoverflow.com/questions/23347287/parameter-pack-expansion-fails
    template<typename T>
    struct get_reference
    {
      typedef typename T::reference reference;
    };

     //typedef std::tuple<typename Iterators::reference...> references;
    typedef std::tuple<typename get_reference<typename unwrap_unref<Iterators>::type>::reference...> references;

    template<typename T>
    empty inc(T& t)
    {
      ++t;
      return empty();
    }

    template<typename T>
    typename T::reference single_deref(T& t)
    {
      return *t;
    }

    template<int ...S>
    references multi_deref(seq<S...>)
    {
      return references(single_deref(std::get<S>(iterators))...);
    }

 
    template<int ...S>
    void multi_inc(seq<S...>)
    {
      ignore(inc(std::get<S>(iterators))...);
    }

  public:
    zip_iterator& operator++()
    {
      multi_inc(tuple_indices());
      return *this;
    }
      
      template<std::size_t I>
      typename std::tuple_element<I, unwrapped_iterators>::type& get_iterator()
      {
          return std::get<I>(iterators);
      }
      template<std::size_t I>
      const typename std::tuple_element<I, unwrapped_iterators>::type& get_iterator() const
      {
          return std::get<I>(iterators);
      }

    template < typename... OtherIterators > //OtherIterators can be different in type of references, etc
    bool operator==(const zip_iterator<OtherIterators...>& that) const
    {
      static_assert(std::is_same
        < pack<typename full_decay<OtherIterators>::type...>
        , pack<typename full_decay<Iterators>::type... > >::value, "incompatible iterators");

//        auto what_is = std::get<0>(that);
//        bool ret_val = (std::get<0>(iterators) == what_is);
//        return (std::get<0>(iterators) == std::get<0>(that));
//        auto val = that.get_iterator<0>();
//        that.get_iterator<0>()
        return std::get<0>(iterators) == std::get<0>(that.iterators);
    }

    template < typename... OtherIterators > //OtherIterators can be different in type of references, etc
    bool operator!=(const zip_iterator<OtherIterators...>& that) const
    {
      static_assert(std::is_same
        < pack<typename full_decay<OtherIterators>::type...>
        , pack<typename full_decay<Iterators>::type... > >::value, "incompatible iterators");

//        return std::get<0>(iterators) != std::get<0>(that);
//      return std::get<0>(iterators) != that.get_iterator<0>();
        return std::get<0>(iterators) != std::get<0>(that.iterators);
    }

    references operator*()
    {
      return multi_deref(tuple_indices());
    }

    

  private:
   unwrapped_iterators iterators;

  };

  template<typename...Iterators>
  zip_iterator<typename std::remove_reference<Iterators>::type...> make_zip_iterator(Iterators&&... its)
  {
    //return zip_iterator<typename std::remove_reference<Iterators>::type...>(announce_pack(), std::forward<Iterators>(its)...);
    return zip_iterator<typename std::remove_reference<Iterators>::type...>(std::forward<Iterators>(its)...);
  }
    
  template<typename...Ranges>
  struct zip_range
  {
  public:
    typedef zip_iterator<typename unwrap_unref<Ranges>::type::iterator...> iterator;

  private:
    typedef std::tuple<typename unwrap<Ranges>::type...> unwrapped_ranges;
    typedef typename gens<sizeof...(Ranges)>::type tuple_indices;

  public:

    template<typename... InRanges>
    explicit zip_range(InRanges&&... rgs) : ranges(std::forward<InRanges>(rgs)...)
    { }

    iterator begin() const
    {
      return multi_begin(tuple_indices());
    }

    iterator end() const 
    {
      return multi_end(tuple_indices());
    }


  private:
    template<typename T> typename T::iterator single_begin(T& t) const
    {
      return t.begin();
    }

    template<typename T> typename T::iterator single_end(T& t) const
    {
      return t.end();
    }

    template<int ...S> iterator multi_begin(seq<S...>) const
    {
      return iterator(single_begin(std::get<S>(ranges))...);
    }

    template<int ...S> iterator multi_end(seq<S...>) const
    {
      return iterator(single_end(std::get<S>(ranges))...);
    }

    unwrapped_ranges ranges;
  };

  template<typename...Ranges>
  zip_range<typename std::remove_reference<Ranges>::type...> make_zip_range(Ranges&&... rgs)
  {
    return zip_range<typename std::remove_reference<Ranges>::type...>(std::forward<Ranges>(rgs)...);
  }


  //Iterate over a range of ranges
  // TODO: this is not being used at teh moment, but should come in handy for 
  // distance weighted moving windows.
   
  template<typename RangeRange>
  struct range_range_iterator
  {

    using range = typename RangeRange::value_type;
    using value_type = typename range::value_type;
    using range_iterator = typename range::iterator;
    using range_reference = typename range_iterator::reference;
    using iterators = std::vector<range_iterator>;
    struct reference_proxy
    {
      void operator=(const value_type& v) const
      {
        **m_iter = v;
      }

      void operator=(const reference_proxy& that) const
      {
        **m_iter = static_cast<value_type>(**that.m_iter);
      }


      // conversion to make the iterator readable
      operator value_type() const
      {
        return **m_iter;;
      }

      reference_proxy(const range_iterator* iter) :m_iter(iter)
      {}

      const range_iterator* m_iter;
    };

    using reference = std::vector<reference_proxy>;


    void find_begin(RangeRange& ranges)
    {
      m_iterators.clear();
      for (auto& range : ranges)
      {
        m_iterators.push_back(range.begin());
      }
    }

    void find_end(RangeRange& ranges)
    {
      m_iterators.clear();
      for (auto& range : ranges)
      {
        m_iterators.push_back(range.end());
      }
    }


    range_reference get(int i)
    {
      return *(m_iterators[i]);

    }
    reference operator*()
    {
      reference result;
      for (int i = 0; i < m_iterators.size(); ++i)
      {
        result.push_back(reference_proxy(&m_iterators[i]));
      }
      return result;
    }

    range_range_iterator& operator++()
    {
      for (int i = 0; i < m_iterators.size(); ++i)
      {
        ++(m_iterators[i]);
      }
      return *this;
    }

    bool operator==(const range_range_iterator& that)
    {
      return m_iterators.size() == 0 || m_iterators[0] == that.m_iterators[0];
    }

    bool operator!=(const range_range_iterator& that)
    {
      return m_iterators.size() != 0 && m_iterators[0] != that.m_iterators[0];
    }

    iterators m_iterators;

  };

  template<typename RangeRange>
  struct range_zip_range
  {
    range_zip_range(RangeRange& range_range) : m_range_range(range_range)
    {}

    using iterator = range_range_iterator<RangeRange>;

    iterator begin()
    {
      iterator i;
      i.find_begin(m_range_range);
      return i;
    }

    iterator end()
    {
      iterator i;
      i.find_end(m_range_range);
      return i;
    }

    RangeRange& m_range_range;

  };

  template<typename RangeRange>
  range_zip_range<RangeRange> make_range_zip_range(RangeRange& rr)
  {
    return range_zip_range<RangeRange>(rr);
  }

 

} //namespace raster_util

#endif