//
//  Neighbourhood_Impl.cpp
//  move_creek
//
//  Created by Jeffrey Newman on 28/01/2015.
//  Copyright (c) 2015 University of Adelaide. All rights reserved.
//

#include <stdio.h>
#include "Types.h"
#include "Neighbourhood.cpp"

template
boost::shared_ptr<Set> get_neighbourhood(raster_util::gdal_raster<double> &  map, unsigned long i, unsigned long j, unsigned long size);

template
boost::shared_ptr<Set> get_neighbourhood(raster_util::gdal_raster<int> &  map, unsigned long i, unsigned long j, unsigned long size);

template
boost::shared_ptr<Set> find_adjacents(raster_util::gdal_raster<int> &  map, unsigned long i, unsigned long j, unsigned long size);

template
boost::shared_ptr<std::vector<std::pair<int, int> > > find_immediate_adjacents(raster_util::gdal_raster<double> &  map, unsigned long i, unsigned long j);