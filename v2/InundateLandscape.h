//
//  InundateLandscape.h
//  inundate-landscape
//
//  Created by a1091793 on 12/05/2016.
//
//

#ifndef InundateLandscape_h
#define InundateLandscape_h

#include <blink/raster/gdal_raster.h>
#include <boost/shared_ptr.hpp>
#include <boost/optional.hpp>

namespace raster_util = blink::raster;
typedef boost::shared_ptr<raster_util::gdal_raster<double> > DoubleRasterSPtr;
typedef boost::shared_ptr<raster_util::gdal_raster<int> > IntRasterSPtr;
typedef raster_util::gdal_raster<double> DoubleRaster;
typedef raster_util::gdal_raster<int> IntRaster;
typedef boost::optional< boost::shared_ptr< std::vector<GuageControl> > > GuagesSPtr;
typedef boost::optional< boost::shared_ptr< std::vector<ChannelNode> > > ControlsSPtr;


void
inundateLandscape( DoubleRaster & inundation, DoubleRaster & dem, IntRaster & hydro_connect, Graph & channel_grph, GuagesSPtr guages, ControlsSPtr controls, double outflow_levels = 1.0, double source_levels = 0.0, bool do_interp_hgl = false);

#endif /* InundateLandscape_h */

