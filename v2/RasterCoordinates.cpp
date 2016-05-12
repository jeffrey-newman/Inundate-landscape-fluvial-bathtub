//
//  RasterCoordinates.cpp
//  inundate-landscape
//
//  Created by a1091793 on 12/05/2016.
//
//

#include <stdio.h>
#include "RasterCoordinates.h"


#include <cmath>

    
GeoTransform::GeoTransform()
: x_origin(0), pixel_width(0), x_line_space(0), y_origin(0), pixel_height(0), y_line_space(0)
{
    
}

GeoTransform::GeoTransform(double * adfGeoTransform)
:   x_origin(adfGeoTransform[0]),
pixel_width(adfGeoTransform[1]),
x_line_space(adfGeoTransform[2]),
y_origin(adfGeoTransform[3]),
pixel_height(adfGeoTransform[4]),
y_line_space(adfGeoTransform[5])
{
    
}



std::pair<int, int>
getRasterCoordinates(const double & x_loc, const double & y_loc, const GeoTransform & transform)
{
    std::pair<int, int> coordinates; //(row, col)
    coordinates.first = std::floor(( y_loc - transform.y_origin ) / transform.y_line_space);
    coordinates.second = std::floor(( x_loc - transform.x_origin ) / transform.pixel_width);
    return (coordinates);
}
