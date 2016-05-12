//
//  RasterCoordinates.h
//  VaryingFloodDepth
//
//  Created by a1091793 on 25/09/2015.
//  Copyright © 2015 University of Adelaide. All rights reserved.
//

#ifndef RasterCoordinates_h
#define RasterCoordinates_h

#include <tuple>

struct GeoTransform {
    double x_origin;
    double pixel_width;
    double x_line_space;
    double y_origin;
    double pixel_height;
    double y_line_space;
    
    GeoTransform();
    
    GeoTransform(double * adfGeoTransform);
    
};

std::pair<int, int>
getRasterCoordinates(const double & x_loc, const double & y_loc, const GeoTransform & transform);


#endif /* RasterCoordinates_h */
