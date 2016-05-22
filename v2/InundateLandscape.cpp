#include <iostream>
#include <sstream>
#include <string>
#include <fstream>
#include <map>
#include <list>



#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/progress.hpp>

#include <Eigen/Dense>


#include <blink/iterator/zip_range.h>

#include "Types.h"
#include "RasterCoordinates.h"
#include "Neighbourhood.h"
#include "GraphSearches.h"
#include "PrintGraphsToFile.h"



//#include "GraphSearches.h"
//#include "RasterCoordinates.h"
//#include "PrintGraphsToFile.h"
//#include "ReadGraphsFromFile.h"
#include "IsChannel.h"

#include "InundateLandscape.h"



void
inundateLandscape( DoubleRaster & inundation, DoubleRaster & dem, IntRaster & hydro_connect, IntRaster & mask, Graph & channel_grph, GuagesSPtr guages, ControlsSPtr controls, double outflow_levels, double source_levels, bool do_interp_hgl)
{
    namespace raster_iterator = blink::iterator;
    /****************************************************************************/
    /*       Check maps for consistency                                         */
    /****************************************************************************/
    if (hydro_connect.nCols() != dem.nCols())
    {
        throw std::runtime_error("Number of columns in the two comparison maps non-identical");
    }
    
    if (hydro_connect.nRows() != dem.nRows())
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
        raster_util::coordinate_2d loc(channel_grph[*vp.first].row, channel_grph[*vp.first].col);
        VertexDescriptor v = *vp.first;
        channel_pixels[loc.row].insert(std::make_pair(loc.col, v));
        channel_grph[*vp.first].elevation = dem.get(loc); //update elevation.
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
    
    /****************************************************************************/
    /*       Process guages                                                     */
    /****************************************************************************/
    if(guages)
    {
        //Now place the guages into the channel_graph.
        BOOST_FOREACH(GuageControl & guage, **guages)
        {
            int row, col;
            double gt_data[6];
            double* geotransform = gt_data;
            CPLErr error_status
            = const_cast<GDALDataset*>(dem.get_gdal_dataset())->GetGeoTransform(geotransform);
            
            boost::tie(row, col) = getRasterCoordinates(guage.x_coord, guage.y_coord, GeoTransform(geotransform));
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
                    boost::shared_ptr<Set> nh = get_neighbourhood(dem, row, col, neighd_size);
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
    
    /****************************************************************************/
    /*       Process controls                                                   */
    /****************************************************************************/
    if (controls)
    {
        BOOST_FOREACH(ChannelNode & node, **controls)
        {
            std::cout << "adding control: " << node.node_id << " " << node.type << " " << node.level << "\n";
            channel_grph[idMap[node.node_id]].level = node.level;
            channel_grph[idMap[node.node_id]].type = node.type;
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
    
    
    
    
    
    /******************************************************/
    /* Creating the D matrix used for linear interpolation*/
    /******************************************************/
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
                node.level = _Yu(i) + dem.get(raster_util::coordinate_2d(node.row, node.col));
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
            node.level += dem.get(raster_util::coordinate_2d(node.row, node.col));
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
    
    
    int no_connection = const_cast<GDALRasterBand *>(hydro_connect.get_gdal_band())->GetNoDataValue();
    boost::progress_display show_progress3((hydro_connect.nRows() * hydro_connect.nCols()));
    //    bool hasNoData = hydro_connect_map->HasNoDataValue();
    //    int32_t hglNoDataVal = hydro_connect_map->NoDataValue();
    //    Position channelloc(-9, -9);
    
    
    auto zip = raster_iterator::make_zip_range(std::ref(dem), std::ref(hydro_connect), std::ref(mask), std::ref(inundation));
    for (auto i : zip)
    {
        const double & pixel_elevation = std::get<0>(i);
        const int & channel_id = std::get<1>(i);
        const int & v_mask = std::get<2>(i);
        auto & level = std::get<3>(i);
        
        if (channel_id != no_connection)
        {
            if ( v_mask != 0)
            {
                level = channel_grph[idMap[channel_id]].level ;
                if (pixel_elevation < level)
                {
                    level = level - pixel_elevation;
                }
                else
                {
                    level = 0.0;
                }
            }
        }
        else
        {
            level = 0.0;
        }
        ++show_progress3;
    }
}

	