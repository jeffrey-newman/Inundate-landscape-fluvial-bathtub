//
//  RasterCoordinates.h
//  VaryingFloodDepth
//
//  Created by a1091793 on 25/09/2015.
//  Copyright © 2015 University of Adelaide. All rights reserved.
//

#ifndef RasterCoordinates_h
#define RasterCoordinates_h

#include <cmath>

std::pair<int, int>
getRasterCoordinates(double x_loc, double y_loc, GeoTransform transform)
{
    std::pair<int, int> coordinates; //(row, col)
    coordinates.first = std::floor(( y_loc - transform.y_origin ) / transform.y_line_space);
    coordinates.second = std::floor(( x_loc - transform.x_origin ) / transform.pixel_width);
    return (coordinates);
}


#endif /* RasterCoordinates_h */
