#include <iostream>
#include <sstream>
#include <string>
#include <fstream>
#include <map>
#include <list>

#include <GDAL/gdal.h>

#include <boost/filesystem.hpp>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/graph/undirected_dfs.hpp>
#include <boost/foreach.hpp>
#include <boost/progress.hpp>

#include <Eigen/Dense>

#include "Types.h"
#include "util/gdal_raster.h"
#include "ReadInMap.h"
#include "Print.h"
#include "Neighbourhood.h"
#include "GraphSearches.h"
#include "ReadInControls.h"
#include "RasterCoordinates.h"
#include "PrintGraphsToFile.h"
#include "ReadGraphsFromFile.h"
#include "IsChannel.h"

typedef boost::shared_ptr<raster_util::gdal_raster<double> > DoubleRasterSPtr;

DoubleRasterSPtr inundateLandscape(DoubleRasterSPtr dem, DoubleRasterSPtr hydro_connect, Graph & channel_grph, std::vector<GuageControl> & guages, boost::optional<std::ifstream> & controls_file)
{
    //Check maps for consistency (same dimensions)
    if (hydro_connect->NCols() != dem_map->NCols())
    {
        throw std::runtime_error("Number of columns in the two comparison maps non-identical");
    }
    
    if (hydro_connect->NRows() != dem_map->NRows())
    {
        throw std::runtime_error("Number of rows in the two comparison maps non-identical");
    }
    
    /****************************************************************************/
    /*       Make Structures for looking up node ids and vertex descriptors     */
    /****************************************************************************/
    std::map<int, std::map<int, VertexDescriptor>  > channel_pixels;
    std::pair<VertexIterator, VertexIterator> vp;
    for (vp = boost::vertices(channel_grph); vp.first != vp.second; ++vp.first)
    {
        int i = channel_grph[*vp.first].row;
        int j = channel_grph[*vp.first].col;
        VertexDescriptor v = *vp.first;
        channel_pixels[i].insert(std::make_pair(j, v));
        channel_grph[*vp.first].elevation = dem_map->Get(i, j); //update elevation.
    }
    
    VertexIDMap idMap;
    VertexIterator di, dj;
    for(boost::tie(di, dj) = boost::vertices(channel_grph); di != dj; ++di)
    {
        idMap.insert(std::make_pair(channel_grph[*di].node_id, (*di)));
        if (channel_grph[*di].terminal_type == OUTFLOW)
        {
            if (channel_grph[*di].level == -999)
            {
                channel_grph[*di].level = outflow_levels;
            }
            
        }
        if (channel_grph[*di].terminal_type == SOURCE)
        {
            if (channel_grph[*di].level == -999)
            {
                channel_grph[*di].level = source_levels;
            }
            
        }
    }
    
    
    
    
    //Identify guage nodes from a guage node file
    /********************************************/
    /*       Read in table of Guage  points     */
    /********************************************/
    std::cout << "\n\n*************************************\n";
    std::cout <<     "*  Reading guage points and levels  *\n";
    std::cout <<     "*************************************" << std::endl;
    // Pause to continue. Read in file here.
    if (guage_file != "no_file")
    {
        
        std::vector<GuageControl> guages;
        std::ifstream fs7(guage_table_path.string().c_str());
        if (fs7.is_open())
        {
            GuageParser<std::string::const_iterator> g; // Our grammar
            std::string str;
            getline(fs7, str); // Header line.
            while (getline(fs7, str))
            {
                if (!(str.empty() || str[0] == '#'))
                {
                    GuageControl guage;
                    std::string::const_iterator iter = str.begin();
                    std::string::const_iterator end = str.end();
                    bool r = phrase_parse(iter, end, g, qi::space, guage);
                    
                    if (r && iter == end)
                    {
                        //Place the guage into the vector
                        guages.push_back(guage);
                    }
                    else
                    {
                        std::cout << "-------------------------\n";
                        std::cout << "Parsing failed:\n";
                        std::cout << " At: " << str << "\n";
                        std::cout << "-------------------------\n";
                    }
                }
            }
        }
        
        //Now place the guages into the channel_graph.
        BOOST_FOREACH(GuageControl & guage, guages)
        {
            int row, col;
            boost::tie(row, col) = getRasterCoordinates(guage.x_coord, guage.y_coord, demTransform);
            //Search neighbourhood for nearest creek pixel.
            bool iscreek = false;
            VertexDescriptor creek_vertex;
            boost::tie(iscreek, creek_vertex) = isCreek(row, col, channel_pixels);
            if (iscreek)
            {
                //Find vertex in channel_graph
                for (vp = boost::vertices(channel_grph); vp.first != vp.second; ++vp.first)
                {
                    if (channel_grph[*vp.first].row == row && channel_grph[*vp.first].col == col)
                    {
                        channel_grph[*vp.first].type = GUAGE_CNTRL;
                        channel_grph[*vp.first].level = guage.level;
                        break;
                    }
                }
                if (vp.first == vp.second)
                {
                    std::cerr << " could not find guage in graph" << std::endl;
                }
                
            }
            else // search in neighbourhood for closest pixel
            {
                int neighd_size = 1;
                bool is_found = false;
                do
                {
                    boost::shared_ptr<Set> nh = get_neighbourhood(dem_map, row, col, neighd_size);
                    BOOST_FOREACH(ChannelNode & loc, *nh)
                    {
                        bool iscreek = false;
                        VertexDescriptor creek_vertex;
                        boost::tie(iscreek, creek_vertex) = isCreek(loc.row, loc.col, channel_pixels);
                        if (iscreek)
                        {
                            channel_grph[creek_vertex].type = GUAGE_CNTRL;
                            channel_grph[creek_vertex].level = guage.level;
                            is_found = true;
                            break;
                        }
                    }
                    ++neighd_size;
                } while (neighd_size < 100 && !is_found);
                
                if (neighd_size >= 100)
                {
                    std::cerr << " could not find guage in graph" << std::endl;
                }
            }
        }
    }

    
}


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
    
	if (guage_file != "no_file")
	{
		if (!fs::exists(guage_table_path))
		{
			std::stringstream ss;
			ss << guage_table_path << " does not exist";
			throw std::runtime_error(ss.str());
			return (EXIT_FAILURE);
		}
	}
    // Check file exists
//    if (!fs::exists(fd_file_path))
//    {
//        std::stringstream ss;
//        ss << fd_file_path << " does not exist";
//        throw std::runtime_error(ss.str());
//        return (EXIT_FAILURE);
//    }

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
    

	/**********************************/
	/*         Read in maps           */
	/**********************************/
	std::cout << "\n\n*************************************\n";
	std::cout <<     "*             Read in maps          *\n";
	std::cout <<     "*************************************" << std::endl;
	// Second the elevation data
	std::tuple<boost::shared_ptr<Map_Matrix<double> >, std::string, GeoTransform> gdal_dem_map = read_in_map<double>(dem_file_path, GDT_Float64, NO_CATEGORISATION);
	boost::shared_ptr<Map_Matrix<double> > dem_map(std::get<0>(gdal_dem_map));
	std::string & demWKTprojection(std::get<1>(gdal_dem_map));
	GeoTransform & demTransform(std::get<2>(gdal_dem_map));
    
    std::tuple<Map_Int_SPtr, std::string, GeoTransform> gdal_hydro_connect_map = read_in_map<int32_t>(hydro_paths_file_path, GDT_Int32, NO_CATEGORISATION);
    Map_Int_SPtr hydro_connect_map(std::get<0>(gdal_hydro_connect_map));
    std::string & hydro_connect_WKTprojection(std::get<1>(gdal_hydro_connect_map));
    GeoTransform & hydro_connect_transform(std::get<2>(gdal_hydro_connect_map));

	//Check maps for consistency (same dimensions)
	if (hydro_connect_map->NCols() != dem_map->NCols())
	{
		throw std::runtime_error("Number of columns in the two comparison maps non-identical");
	}

	if (hydro_connect_map->NRows() != dem_map->NRows())
	{
		throw std::runtime_error("Number of rows in the two comparison maps non-identical");
	}
    
    
    /****************************************************************************/
    /*       Make Structures for looking up node ids and vertex descriptors     */
    /****************************************************************************/
    std::map<int, std::map<int, VertexDescriptor>  > channel_pixels;
    std::pair<VertexIterator, VertexIterator> vp;
    for (vp = boost::vertices(channel_grph); vp.first != vp.second; ++vp.first)
    {
        int i = channel_grph[*vp.first].row;
        int j = channel_grph[*vp.first].col;
        VertexDescriptor v = *vp.first;
        channel_pixels[i].insert(std::make_pair(j, v));
		channel_grph[*vp.first].elevation = dem_map->Get(i, j); //update elevation.
    }
    
    VertexIDMap idMap;
    VertexIterator di, dj;
    for(boost::tie(di, dj) = boost::vertices(channel_grph); di != dj; ++di)
    {
        idMap.insert(std::make_pair(channel_grph[*di].node_id, (*di)));
        if (channel_grph[*di].terminal_type == OUTFLOW)
        {
            if (channel_grph[*di].level == -999)
            {
                channel_grph[*di].level = outflow_levels;
            }
            
        }
        if (channel_grph[*di].terminal_type == SOURCE)
        {
            if (channel_grph[*di].level == -999)
            {
                channel_grph[*di].level = source_levels;
            }
            
        }
    }
	
    
    
    
    //Identify guage nodes from a guage node file
    /********************************************/
    /*       Read in table of Guage  points     */
    /********************************************/
    std::cout << "\n\n*************************************\n";
    std::cout <<     "*  Reading guage points and levels  *\n";
    std::cout <<     "*************************************" << std::endl;
    // Pause to continue. Read in file here.
	if (guage_file != "no_file")
	{

		std::vector<GuageControl> guages;
		std::ifstream fs7(guage_table_path.string().c_str());
		if (fs7.is_open())
		{
			GuageParser<std::string::const_iterator> g; // Our grammar
			std::string str;
			getline(fs7, str); // Header line.
			while (getline(fs7, str))
			{
				if (!(str.empty() || str[0] == '#'))
				{
					GuageControl guage;
					std::string::const_iterator iter = str.begin();
					std::string::const_iterator end = str.end();
					bool r = phrase_parse(iter, end, g, qi::space, guage);

					if (r && iter == end)
					{
						//Place the guage into the vector
						guages.push_back(guage);
					}
					else
					{
						std::cout << "-------------------------\n";
						std::cout << "Parsing failed:\n";
						std::cout << " At: " << str << "\n";
						std::cout << "-------------------------\n";
					}
				}
			}
		}

		//Now place the guages into the channel_graph.
		BOOST_FOREACH(GuageControl & guage, guages)
		{
			int row, col;
			boost::tie(row, col) = getRasterCoordinates(guage.x_coord, guage.y_coord, demTransform);
			//Search neighbourhood for nearest creek pixel.
			bool iscreek = false;
			VertexDescriptor creek_vertex;
			boost::tie(iscreek, creek_vertex) = isCreek(row, col, channel_pixels);
			if (iscreek)
			{
				//Find vertex in channel_graph
				for (vp = boost::vertices(channel_grph); vp.first != vp.second; ++vp.first)
				{
					if (channel_grph[*vp.first].row == row && channel_grph[*vp.first].col == col)
					{
						channel_grph[*vp.first].type = GUAGE_CNTRL;
						channel_grph[*vp.first].level = guage.level;
						break;
					}
				}
				if (vp.first == vp.second)
				{
					std::cerr << " could not find guage in graph" << std::endl;
				}

			}
			else // search in neighbourhood for closest pixel
			{
				int neighd_size = 1;
				bool is_found = false;
				do
				{
					boost::shared_ptr<Set> nh = get_neighbourhood(dem_map, row, col, neighd_size);
					BOOST_FOREACH(ChannelNode & loc, *nh)
					{
						bool iscreek = false;
						VertexDescriptor creek_vertex;
						boost::tie(iscreek, creek_vertex) = isCreek(loc.row, loc.col, channel_pixels);
						if (iscreek)
						{
							channel_grph[creek_vertex].type = GUAGE_CNTRL;
							channel_grph[creek_vertex].level = guage.level;
							is_found = true;
							break;
						}
					}
					++neighd_size;
				} while (neighd_size < 100 && !is_found);

				if (neighd_size >= 100)
				{
					std::cerr << " could not find guage in graph" << std::endl;
				}
			}
		}
	}

    
	if (controls_file != "no_file" && use_controls_file)
    {
        /********************************************/
        /*      Read in table of control points     */
        /********************************************/
        std::cout << "\n\n*************************************\n";
        std::cout <<     "*  Reading table of control points  *\n";
        std::cout <<     "*************************************" << std::endl;
        // Pause to continue. Read in file here.
        fs::path controls_path(controls_file);
        
        if (!fs::exists(controls_path))
        {
            std::stringstream ss;
            ss << controls_path << " does not exist";
            throw std::runtime_error(ss.str());
            return (EXIT_FAILURE);
        }

        
        std::ifstream fs4(controls_path.string().c_str());
        if (fs4.is_open())
        {
            
            ControlParser<std::string::const_iterator> g; // Our grammar
            std::string str;
            getline(fs4, str); // Header line.
            while (getline(fs4, str))
            {
                if (!(str.empty() || str[0] == '#'))
                {
                    ChannelNode node;
                    std::string::const_iterator iter = str.begin();
                    std::string::const_iterator end = str.end();
                    bool r = phrase_parse(iter, end, g, qi::space, node);
                    
                    if (r && iter == end)
                    {
                        //Place the level into the appropriate node;
                        if (node.type > 1)
                        {
							std::cout << "adding control: " << node.node_id << " " << node.type << " " << node.level << "\n";
                            channel_grph[idMap[node.node_id]].level = node.level;
                            channel_grph[idMap[node.node_id]].type = node.type;
                        }
                        
                    }
                    else
                    {
                        std::cout << "-------------------------\n";
                        std::cout << "Parsing failed:\n";
                        std::cout << " At: " << str << "\n";
                        std::cout << "-------------------------\n";
                    }
                }
            }
            fs4.close();
        }
    }

	if (write_controls)
	{

		/********************************************/
		/*       Print table of control points      */
		/********************************************/
		std::cout << "\n\n*************************************\n";
		std::cout << "*  Writing table of control points  *\n";
		std::cout << "*************************************" << std::endl;
		// Iterate through the vertices and print them out
		std::ofstream fs3("controls.txt");
		if (fs3.is_open())
		{
			fs3 << "# id\trow\tcol\ttype\tlevel\n";
			fs3 << "# Control nodes:\n";
			std::pair<VertexIterator, VertexIterator> vp;
			for (vp = boost::vertices(channel_grph); vp.first != vp.second; ++vp.first)
			{
				if (channel_grph[*(vp.first)].type > 1)
				{
					fs3 << channel_grph[*(vp.first)].node_id << "\t" << channel_grph[*(vp.first)].row << "\t" << channel_grph[*(vp.first)].col << "\t" << channel_grph[*(vp.first)].type << "\t" <<
						channel_grph[*(vp.first)].level << std::endl;
				}
			}
			fs3 << "# Non control nodes:\n";
			for (vp = boost::vertices(channel_grph); vp.first != vp.second; ++vp.first)
			{
				if (channel_grph[*(vp.first)].type == 1)
				{
					fs3 << channel_grph[*(vp.first)].node_id << "\t" << channel_grph[*(vp.first)].row << "\t" << channel_grph[*(vp.first)].col << "\t" << channel_grph[*(vp.first)].type << "\t" <<
						channel_grph[*(vp.first)].level << std::endl;
				}
			}
			fs3.close();
		}
	}

    /********************************************/
    /*        Create control node graph         */
    /********************************************/
    std::cout << "\n\n*************************************\n";
    std::cout <<     "*    Creating control node graph    *\n";
    std::cout <<     "*************************************" << std::endl;
    
	for (vp = boost::vertices(channel_grph); vp.first != vp.second; ++vp.first)
	{
		channel_grph[*(vp.first)].up_cntrl_id = -1;
		channel_grph[*(vp.first)].up_cntrl_dist = -1;
		channel_grph[*(vp.first)].down_cntrl_id = -1;
		channel_grph[*(vp.first)].down_cntrl_dist = -1;
	}

    Graph controls_grph;
    VertexIDMap node_id_map;
//    std::pair<VertexIterator, VertexIterator> vp;
    for (vp = boost::vertices(channel_grph); vp.first != vp.second; ++vp.first)
    {
        if (channel_grph[*(vp.first)].type > 1)
        {
            VertexDescriptor v = boost::add_vertex(controls_grph);
            controls_grph[v] = channel_grph[*(vp.first)];
            node_id_map.insert(std::make_pair(channel_grph[*(vp.first)].node_id, v));
            channel_grph[*(vp.first)].up_cntrl_id = channel_grph[*(vp.first)].node_id;
            channel_grph[*(vp.first)].up_cntrl_dist = 1;
            channel_grph[*(vp.first)].down_cntrl_id = channel_grph[*(vp.first)].node_id;
            channel_grph[*(vp.first)].down_cntrl_dist = 1;
        }
    }
    
    // Create links in controls graph using a search across the channels graph at each point.
    for (vp = boost::vertices(controls_grph); vp.first != vp.second; ++vp.first)
    {
        int control_id = controls_grph[*(vp.first)].node_id;
        VertexDescriptor channel_vertex = idMap[control_id];
        controlSearch(channel_grph, channel_vertex, controls_grph, *(vp.first), node_id_map);
    }
    
    
    
    
    
	/********************************************/
	/* Creating the D matrix */
	/********************************************/
	std::cout << "\n\n*************************************\n";
	std::cout <<     "*       Creating the D matrix       *\n";
	std::cout <<     "*************************************" << std::endl;
    //Sort into
    int cntrl_node_id = 0;
    VertexIDMap cntrls_node_id_map;
    for (vp = boost::vertices(controls_grph); vp.first != vp.second; ++vp.first)
    {
        if (controls_grph[*(vp.first)].level == -999)
        {
            controls_grph[*(vp.first)].cntrl_node_id = cntrl_node_id;
            cntrls_node_id_map.insert(std::make_pair(cntrl_node_id, *(vp.first)));
            ++cntrl_node_id;
        }
    }
    
    int num_unknown_cntrls = cntrl_node_id;
    int num_cntrls = boost::num_vertices(controls_grph);
    int num_known_cntrls = num_cntrls - num_unknown_cntrls;
    
    Eigen::VectorXd Yc(num_known_cntrls);
    int i = 0;
    for (vp = boost::vertices(controls_grph); vp.first != vp.second; ++vp.first)
    {
        if (controls_grph[*(vp.first)].level != -999)
        {
            Yc(i) = controls_grph[*(vp.first)].level;
            controls_grph[*(vp.first)].cntrl_node_id = cntrl_node_id;
            cntrls_node_id_map.insert(std::make_pair(cntrl_node_id, *(vp.first)));
            ++cntrl_node_id;
            ++i;
            
        }
    }
    
    //std::cout << "Here is the vector Yc:\n" << Yc << std::endl;
    
    Eigen::MatrixXd _D = Eigen::MatrixXd::Zero(num_cntrls,num_cntrls);
    
    for (vp = boost::vertices(controls_grph); vp.first != vp.second; ++vp.first, ++cntrl_node_id)
    {
        int node_id = controls_grph[*(vp.first)].cntrl_node_id;
        double sum_inv_dist = 0;
        OutEdgeIterator edge, end;
        for(boost::tie(edge, end) = boost::out_edges(*(vp.first), controls_grph); edge != end; ++edge)
        {
            VertexDescriptor v = boost::target(*edge, controls_grph);
            int neighbour_id = controls_grph[v].cntrl_node_id;
            double inv_dist = 1 / double(controls_grph[*edge].distance);
            _D(node_id,neighbour_id) = -1 * inv_dist;
            sum_inv_dist += inv_dist;
        }
        _D(node_id, node_id) = sum_inv_dist;
    }
    
    
    
    
    
    /********************************************/
    /*            Solve the system              */
    /********************************************/
//    Eigen::MatrixXd _D1(num_unknown_cntrls,num_unknown_cntrls)
    Eigen::MatrixXd _D1
        = _D.block(0,0,num_unknown_cntrls,num_unknown_cntrls);
//    Eigen::MatrixXd _D2(num_unknown_cntrls,num_known_cntrls)
    Eigen::MatrixXd _D2
        = _D.block(0,num_unknown_cntrls,num_unknown_cntrls,num_known_cntrls);
//    Eigen::MatrixXd _D3(num_known_cntrls,num_unknown_cntrls)
    Eigen::MatrixXd _D3
        = _D.block(num_unknown_cntrls,0,num_known_cntrls,num_unknown_cntrls);
//    Eigen::MatrixXd _D4(num_known_cntrls,num_known_cntrls)
    Eigen::MatrixXd _D4
        = _D.block(num_unknown_cntrls,num_unknown_cntrls,num_known_cntrls,num_known_cntrls);

    Eigen::VectorXd _M = -1 * (_D2 * Yc);
    
//    std::cout << "Here is the matrix D1:\n" << _D1 << std::endl;
//    std::cout << "Here is the vector M:\n" << _M << std::endl;
	if (_D1.rows() != 0)
	{

		Eigen::VectorXd _Yu = _D1.colPivHouseholderQr().solve(_M);
		//    std::cout << "The solution is:\n" << _Yu << std::endl;

		std::ofstream fs_matrix("matrices.txt");
		if (fs_matrix.is_open())
		{
			fs_matrix << "D1 = \n" << _D1 << "\n\n\n";
			fs_matrix << "M = \n" << _M << "\n\n\n";
			fs_matrix << "Yu = \n" << _Yu << "\n\n\n";
			fs_matrix << "D = \n" << _D << "\n\n\n";
			fs_matrix << "Yc = \n" << Yc << "\n\n\n";
			fs_matrix.close();
		}

		/****************************************************/
		/*  Place levels in controls within channels graph  */
		/****************************************************/
		for (int i = 0; i < num_unknown_cntrls; ++i)
		{
			int node_id = controls_grph[cntrls_node_id_map[i]].node_id;
			controls_grph[cntrls_node_id_map[i]].level = _Yu(i);
			ChannelNode & node = channel_grph[idMap[node_id]];
			if (do_interp_hgl)
			{
				node.level = _Yu(i) + dem_map->Get(node.row, node.col);
				node.inundation = _Yu(i);
			}
			else
			{
				node.level = _Yu(i);
				node.inundation = _Yu(i);
			}

		}
	}
    
    /****************************************************/
    /*  Linearly interpolate all channel pixel levels   */
    /****************************************************/
    for (vp = boost::vertices(channel_grph); vp.first != vp.second; ++vp.first)
    {
        ChannelNode & node = channel_grph[*vp.first];
        double up_dist = 1.0 / double(node.up_cntrl_dist);
        double down_dist = 1.0 / double(node.down_cntrl_dist);
        node.level = (down_dist*channel_grph[idMap[node.down_cntrl_id]].level
                        + up_dist*channel_grph[idMap[node.up_cntrl_id]].level)
                            / (up_dist + down_dist);
    }

	/*******************************************************/
	/*  Fix up so that inundation is water depth           */
	/*                 level is elevation of water surface */
	/*                 elevation is ground elevation       */
	/*******************************************************/
	if (!do_interp_hgl)
	{
		for (vp = boost::vertices(channel_grph); vp.first != vp.second; ++vp.first)
		{
			ChannelNode & node = channel_grph[*vp.first];
			node.inundation = node.level;
			node.level += dem_map->Get(node.row, node.col);
			if (node.level < 0) node.level = 0;
		}
	}
	else
	{
		for (vp = boost::vertices(channel_grph); vp.first != vp.second; ++vp.first)
		{
			ChannelNode & node = channel_grph[*vp.first];
			node.inundation = node.level - node.elevation;
			if (node.level < 0) node.level = 0;
		}
		
	}
	
    
    /********************************************/
    /*       Print Control graphs to file       */
    /********************************************/
    std::cout << "\n\n*************************************\n";
    std::cout <<     "*  Printing control Graph to file   *\n";
    std::cout <<     "*************************************" << std::endl;
    std::string controls_file_name = "control_graph";
    printGraphsToFile(controls_grph, controls_file_name);
    
    /********************************************/
    /*       Print Channel graphs to file       */
    /********************************************/
    std::cout << "\n\n*************************************\n";
    std::cout <<     "*  Printing channel Graph to file   *\n";
    std::cout <<     "*************************************" << std::endl;
    std::string filename = "channel_graph";
    printGraphsToFile(channel_grph, filename);
    
    
	
    /*******************************************************************/
    /*     Assign height of water column according to water height at connected channel pixel  */
    /*******************************************************************/
    std::cout << "\n\n*************************************************\n";
    std::cout <<     "*   Assign water column height at each pixel    *\n";
    std::cout <<    "**************************************************" << std::endl;
    //Create output map.
    double null_value = 0;
    //if (dem_map->HasNoDataValue()) null_value = dem_map->NoDataValue();
    Map_Double_SPtr output_map(new Map_Double(dem_map->NRows(), dem_map->NCols(), null_value));
    output_map->SetNoDataValue(null_value);
    
    int32_t no_connection = hydro_connect_map->NoDataValue();
    boost::progress_display show_progress3((hydro_connect_map->NRows()*hydro_connect_map->NCols()));
//    bool hasNoData = hydro_connect_map->HasNoDataValue();
//    int32_t hglNoDataVal = hydro_connect_map->NoDataValue();
//    Position channelloc(-9, -9);
    
    for (unsigned int i = 0; i < hydro_connect_map->NRows(); ++i)
    {
        for (unsigned int j = 0; j < hydro_connect_map->NCols(); ++j)
        {
            int channel_id = hydro_connect_map->Get(i, j);
            if (channel_id != no_connection)
            {
//                channelloc = channel_pixels_vec[channel_id];
//                if (channelloc.first != -9 && channelloc.second != -9)
//                {
                    double level = channel_grph[idMap[channel_id]].level ;

                    double pixel_elevation = dem_map->Get(i, j);
                    if (pixel_elevation < level)
                    {
                        output_map->Get(i, j) = level - pixel_elevation;
                    }
            }
            ++show_progress3;
        }
    }
    
    /********************************************/
    /*    CA for the  oversubcatchment flow     */
    /********************************************/
    //Make list of pixels in CA Domain.
    //This is pixels with flood-depth = NaN and elevation != NaN.
    //std::vector<std::pair<int, int> > ca_pixels;
    //for (unsigned int i = 0; i < output_map->NRows(); ++i)
    //{
    //    for (unsigned int j = 0; j < output_map->NCols(); ++j)
    //    {
    //        if (output_map->Get(i,j) == output_map->NoDataValue() && dem_map->Get(i, j) != dem_map->NoDataValue())
    //        {
    //            ca_pixels.push_back(std::make_pair(i, j));
    //        }
    //    }
    //}
    //
    //First transition rule - eliminate neighbour cells.
    
    
    
    
    
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

	