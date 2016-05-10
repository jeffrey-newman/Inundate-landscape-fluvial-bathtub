//
//=======================================================================
// Copyright 2015
// Author: Alex Hagen-Zanker
// University of Surrey
//
// Distributed under the MIT Licence (http://opensource.org/licenses/MIT)
//=======================================================================
//
// Defines exception to be thrown by the moving_window library.
//

#ifndef MOVING_WINDOW_EXCEPTIONS_H_AHZ
#define MOVING_WINDOW_EXCEPTIONS_H_AHZ

#include <boost/exception/all.hpp>

namespace raster_util {

  struct creating_a_raster_failed : public boost::exception, public std::exception
  {
    const char *what() const _NOEXCEPT { return "creating a raster failed"; }
  };

  struct insufficient_memory_for_raster_block : public boost::exception, public std::exception
  {
    const char *what() const _NOEXCEPT { return "insufficient memory for reading a raster block"; }
  };

  struct opening_raster_failed : public boost::exception, public std::exception
  {
    const char *what() const _NOEXCEPT { return "opening raster failed"; }
  };
  struct reading_from_raster_failed : public boost::exception, public std::exception
  {
    const char *what() const _NOEXCEPT { return "reading from raster failed"; }
  };

  struct writing_to_raster_failed : public boost::exception, public std::exception
  {
    const char *what() const _NOEXCEPT { return "writing to raster failed"; }
  };

} //namespace raster_util 
#endif