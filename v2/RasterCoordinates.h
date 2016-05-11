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

struct GeoTransform {
    double x_origin;
    double pixel_width;
    double x_line_space;
    double y_origin;
    double pixel_height;
    double y_line_space;
    
    GeoTransform()
    : x_origin(0), pixel_width(0), x_line_space(0), y_origin(0), pixel_height(0), y_line_space(0)
    {
        
    }
    
    GeoTransform(double * adfGeoTransform)
    :   x_origin(adfGeoTransform[0]),
    pixel_width(adfGeoTransform[1]),
    x_line_space(adfGeoTransform[2]),
    y_origin(adfGeoTransform[3]),
    pixel_height(adfGeoTransform[4]),
    y_line_space(adfGeoTransform[5])
    {
        
    }
    
};

std::pair<int, int>
getRasterCoordinates(const double & x_loc, const double & y_loc, const GeoTransform & transform)
{
    std::pair<int, int> coordinates; //(row, col)
    coordinates.first = std::floor(( y_loc - transform.y_origin ) / transform.y_line_space);
    coordinates.second = std::floor(( x_loc - transform.x_origin ) / transform.pixel_width);
    return (coordinates);
}


#endif /* RasterCoordinates_h */
