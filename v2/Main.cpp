//
//  Main.cpp
//  inundate-landscape
//
//  Created by a1091793 on 10/05/2016.
//
//

#include <stdio.h>


int main(int argc, char **argv)
{
    
    
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
    
    namespace prog_opt = boost::program_options;
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

    
    
    /********************************************/
    /* Print resultent DEM					    */
    /********************************************/
    std::cout << "\n\n*************************************\n";
    std::cout << "*         Saving output file        *\n";
    std::cout << "*************************************" << std::endl;
    std::string driverName = "GTiff";
    write_map(output_file_path, GDT_Float64, output_map, demWKTprojection, demTransform, driverName);
    
    return (EXIT_SUCCESS);
    //	double no_changes_val = 0.0;
    //	if (dem_map->HasNoDataValue()) no_changes_val = dem_map->NoDataValue();
    //	Map_Double_SPtr changes_map(new Map_Double(feature_map->NRows(), feature_map->NCols(), no_changes_val));
    //	changes_map->SetNoDataValue(no_changes_val);
    
    
    //	boost::progress_display show_progress3(boost::num_vertices(channel_grph));
    //	dfs_interpolate_visitor vis(dem_map, changes_map, show_progress3);
    //	boost::depth_first_search(channel_grph, visitor(vis));
    
    /********************************************/
    /* //Create map of downstream control IDs, upstream control IDs, Distance to downstram and upstream IDs					    */
    /********************************************/
    //	std::string driverName = "GTiff";
    //	write_map(incised_dem_file_path, GDT_Float64, dem_map, demWKTprojection, demTransform, driverName);
    //	write_map(changes_file_path, GDT_Float64, changes_map, demWKTprojection, demTransform, driverName);
    
}

