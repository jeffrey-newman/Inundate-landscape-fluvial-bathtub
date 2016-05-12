//
//  Main.cpp
//  inundate-landscape
//
//  Created by a1091793 on 10/05/2016.
//
//

#include <stdio.h>
#include <string>
#include <iostream>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <blink/raster/utility.h> // To open rasters
#include "ReadInControlsAndGuages.h"
#include "ReadGraphsFromFile.h"
#include "InundateLandscape.cpp"


int main(int argc, char **argv)
{
    namespace prog_opt = boost::program_options;
    namespace fs = boost::filesystem;
    namespace raster_util = blink::raster;
    
    /**********************************/
    /*        Program options         */
    /**********************************/
    // Need to specify elevation grid
    // Need to specify channel
    //	std::string feature_file;
    std::string dem_file;
    std::string guage_file("no_file");
    //	std::string changes_file;
    //	std::string log_file;
    //    std::string fd_file;
    std::string hydro_paths_file;
    std::string output_file;
    std::string channel_graph_file;
    std::string controls_file("no_file");
    //	bool do_print = false;
    //	std::string file_print;
    //    unsigned int trim_level;
    bool do_interp_hgl = true;
    bool write_controls= false;
    double outflow_levels = 1.0;
    double source_levels = 0.0;
    bool use_controls_file = false;
    
    
    prog_opt::options_description desc("Allowed options");
    desc.add_options()
    ("help,h", "produce help message")
    //		("feature-map,f", prog_opt::value<std::string>(&feature_file), "path of the gdal capatible raster feature file")
    ("dem-map,d", prog_opt::value<std::string>(&dem_file), "path of the gdal capatible elevation data file")
    //        ("flow-dir-map,i", prog_opt::value<std::string>(&fd_file), "path of the gdal capatible flow direction data file")
    ("hydro-paths-file,p", prog_opt::value<std::string>(&hydro_paths_file)->default_value("hydro-paths.tif"), "path of the output map where each pixel is assigned the location on channel that the pixel is hydrologically connected to ")
    ("channel-graph,g", prog_opt::value<std::string>(&channel_graph_file), "path of the graphml representation of the channel")
    ("guage-table,t", prog_opt::value<std::string>(&guage_file), "path of text file with guage location and levels, takes precedence over channel graph")
    ("output-flood-height-file,o", prog_opt::value<std::string>(&output_file)->default_value("flood_height.tif"), "path of the output map where each pixel is assigned the flood height at that pixel")
    //        ("trim-branches,t", prog_opt::value<unsigned int>(&trim_level)->default_value(0), "remove branches if they are composed with a number of pixels less than this amount")
    ("interp-hgl,n", prog_opt::value<bool>(&do_interp_hgl)->default_value(true), "True for interpolation based on hgl, false if based on water column depth")
    ("write-controls,w", prog_opt::value<bool>(&write_controls)->default_value(false), "Write list of control nodes to file")
    ("read-controls,r", prog_opt::value<std::string>(&controls_file), "Name of controls file to read into. Take precedence over guage table file")
    ("outflow-levels,u", prog_opt::value<double>(&outflow_levels), "Flood level assigned to outflow nodes (least precedence)")
    ("source-levels,s", prog_opt::value<double>(&source_levels), "Flood level assigned to outflow nodes (least precedence)");
    //		("log_file,l", prog_opt::value<std::string>(&log_file)->default_value("moves.log"), "path of the file which logs feature cell movements")
    //		("print,p", prog_opt::value<std::string>(&file_print), "Print map to text file (space seperated values) - specify file name prefix");
    
    prog_opt::variables_map vm;
    prog_opt::store(prog_opt::parse_command_line(argc, argv, desc), vm);
    prog_opt::notify(vm);
    if (vm.count("help"))
    {
        std::cout << desc << "\n";
        return 1;
    }
    if (vm.count("read-controls"))
    {
        use_controls_file = true;
    }
    //	if (vm.count("print"))
    //	{
    //		do_print = true;
    //	}
    
    
    fs::path channel_graph_path(channel_graph_file);
    fs::path dem_file_path(dem_file);
    //	fs::path changes_file_path(changes_file);
    fs::path guage_table_path(guage_file);
    //  fs::path fd_file_path(fd_file);
    fs::path hydro_paths_file_path(hydro_paths_file);
    fs::path output_file_path(output_file);
    fs::path controls_file_path(controls_file);

    
    
    // Check file exists
    if (!fs::exists(channel_graph_path))
    {
        std::stringstream ss;
        ss << channel_graph_path << " does not exist";
        throw std::runtime_error(ss.str());
        return (EXIT_FAILURE);
    }
    
    if (!fs::exists(dem_file_path))
    {
        std::stringstream ss;
        ss << dem_file_path << " does not exist";
        throw std::runtime_error(ss.str());
        return (EXIT_FAILURE);
    }
    
    if (!fs::exists(hydro_paths_file_path))
    {
        std::stringstream ss;
        ss << hydro_paths_file_path << " does not exist";
        throw std::runtime_error(ss.str());
        return (EXIT_FAILURE);
    }
    
    
    GuagesSPtr guages;
    if (guage_file != "no_file")
    {
        if (!fs::exists(guage_table_path))
        {
            std::stringstream ss;
            ss << guage_table_path << " does not exist";
            throw std::runtime_error(ss.str());
            return (EXIT_FAILURE);
        }
        
        guages = readInGuages(guage_table_path);
    }
    
    ControlsSPtr controls;
    if(use_controls_file)
    {
        if (!fs::exists(controls_file_path))
        {
            std::stringstream ss;
            ss << controls_file_path << " does not exist";
            throw std::runtime_error(ss.str());
            return (EXIT_FAILURE);
        }
        controls = readInControls(controls_file_path);
    }
    
    
    /**********************************/
    /*       Create graph object      */
    /**********************************/
    Graph channel_grph;
    
    
    /**********************************/
    /*         Read in Graph           */
    /**********************************/
    std::cout << "\n\n*************************************\n";
    std::cout <<     "*             Read in Graphs          *\n";
    std::cout <<     "*************************************" << std::endl;
    //    readGraphFromFile(control_graph_path, control_grph);
    readGraphFromFile(channel_graph_path, channel_grph);

    auto dem = raster_util::open_gdal_raster<double>(dem_file_path.string(), GA_ReadOnly);
    auto hydro_connect = raster_util::open_gdal_raster<int>(hydro_paths_file_path.string(), GA_ReadOnly);
    auto inundation = raster_util::create_gdal_raster_from_model<double>(output_file, dem);
    const_cast<GDALRasterBand *>(inundation.get_gdal_band())->SetNoDataValue(0.0);
    
    inundateLandscape(inundation, dem, hydro_connect, channel_grph, guages, controls);
    
    return (EXIT_SUCCESS);

    
}

