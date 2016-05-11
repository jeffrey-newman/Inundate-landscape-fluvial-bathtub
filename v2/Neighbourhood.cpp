//
//  neighbourhood.cpp
//  move_creek
//
//  Created by a1091793 on 23/01/2015.
//  Copyright (c) 2015 University of Adelaide. All rights reserved.
//

#include "Neighbourhood.h"
#include <stdio.h>
#include <cmath>
#include <algorithm>
#include <blink/raster/gdal_raster.h>
//#include <blink/raster/coordinate_2d.h>

namespace raster_util = blink::raster;

template <typename DataType>
boost::shared_ptr<Set> get_neighbourhood(raster_util::gdal_raster<DataType> & map, unsigned long i, unsigned long j, unsigned long size)
{
    int ii = (int) i;
    int jj = (int) j;
    int sizei = (int) size;
    boost::shared_ptr<Set> neighbourhood(new Set);
    int pbsuccess; DataType noData = const_cast<GDALRasterBand *>(map.get_gdal_band())->GetNoDataValue(&pbsuccess);
    
    
    int end_y = (int) std::min(ii+sizei, (int) (map.nRows() - 1) );
    int begin_y = (int) std::max(ii-sizei, 0);
    
    int end_x = (int) std::min(jj+sizei, (int) (map.nCols() - 1) );
    int begin_x = (int) std::max(jj-sizei, 0);
    
    for (int ti = begin_y; ti <= end_y; ++ti)
    {
        for (int tj = begin_x; tj <= end_x; ++tj)
        {
            if (euclidean_distance(ti, tj, ii, jj) < double(sizei) && map.get(raster_util::coordinate_2d(ti, tj)) != noData )
            {
                ChannelNode node;
                node.row = ti;
                node.col = tj;
                neighbourhood->push_back(node);
            }
        }
    }
    return (neighbourhood);
}

template <typename DataType>
boost::shared_ptr<Set> find_adjacents(raster_util::gdal_raster<DataType> & map, unsigned long i, unsigned long j, unsigned long size)
{
    int ii = (int) i;
    int jj = (int) j;
    int radius = (int) size;
    boost::shared_ptr<Set> adjacents(new Set);
    int pbsuccess; DataType noData = const_cast<GDALRasterBand *>(map.get_gdal_band())->GetNoDataValue(&pbsuccess);
    
	int end_y = (int)std::min(ii + radius, (int)(map.nRows() - 1));
	int begin_y = (int)std::max(ii - radius, 0);
    
	int end_x = (int)std::min(jj + radius, (int)(map.nCols() - 1));
	int begin_x = (int)std::max(jj - radius, 0);
    
    for (int ti = begin_y; ti <= end_y; ++ti)
    {
        for (int tj = begin_x; tj <= end_x; ++tj)
        {
            DataType val = (DataType) map.get(raster_util::coordinate_2d(ti, tj));
			//double dist = euclidean_distance(ti, tj, ii, jj);
			//if (dist < (double) radius && val != noData) adjacents->push_back(std::make_pair(ti - ii, tj - jj));
            ChannelNode node;
            node.row = ti;
            node.col = tj;
			if (val != noData) adjacents->push_back(node);
        }
    }
    return (adjacents);
}

template <typename DataType>
boost::shared_ptr<std::vector<std::pair<int, int> > > find_immediate_adjacents(raster_util::gdal_raster<DataType> & map, unsigned long i, unsigned long j)
{
    int ii = (int) i;
    int jj = (int) j;
    int radius = 1;
    boost::shared_ptr<std::vector<std::pair<int, int> > > adjacents(new std::vector<std::pair<int, int> >);
    int pbsuccess; DataType noData = const_cast<GDALRasterBand *>(map.get_gdal_band())->GetNoDataValue(&pbsuccess);
    
    int end_y = (int)std::min(ii + radius, (int)(map.nRows() - 1));
    int begin_y = (int)std::max(ii - radius, 0);
    
    int end_x = (int)std::min(jj + radius, (int)(map.nCols() - 1));
    int begin_x = (int)std::max(jj - radius, 0);
    
    for (int ti = begin_y; ti <= end_y; ++ti)
    {
        for (int tj = begin_x; tj <= end_x; ++tj)
        {
            //double dist = euclidean_distance(ti, tj, ii, jj);
            //if (dist < (double) radius && val != noData) adjacents->push_back(std::make_pair(ti - ii, tj - jj));
            if ((DataType) map.get(raster_util::coordinate_2d(ti, tj)) != noData) adjacents->push_back(std::make_pair(ti,tj));
        }
    }
    return (adjacents);
}
