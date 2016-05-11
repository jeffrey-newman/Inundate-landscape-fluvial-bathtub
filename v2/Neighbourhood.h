//
//  neighbourhood.h
//  move_creek
//
//  Created by a1091793 on 23/01/2015.
//  Copyright (c) 2015 University of Adelaide. All rights reserved.
//

#ifndef __move_creek__neighbourhood__
#define __move_creek__neighbourhood__

#include "Types.h"
#include <blink/raster/gdal_raster.h>

inline double
euclidean_distance(int i_x, int i_y, int j_x, int j_y)
{
    int dx = abs(i_x - j_x);
    int dy = abs(i_y - j_y);
    return (std::sqrt((double)(dx*dx + dy*dy)));
}

namespace raster_util = blink::raster;

template <typename DataType>
boost::shared_ptr<Set> get_neighbourhood(raster_util::gdal_raster<DataType> & map, unsigned long i, unsigned long j, unsigned long size);

template <typename DataType>
boost::shared_ptr<Set> find_adjacents(raster_util::gdal_raster<DataType> & map, unsigned long i, unsigned long j, unsigned long size);

template <typename DataType>
boost::shared_ptr<std::vector<std::pair<int, int> > > find_immediate_adjacents(raster_util::gdal_raster<DataType> & map, unsigned long i, unsigned long j);


#endif /* defined(__move_creek__neighbourhood__) */
