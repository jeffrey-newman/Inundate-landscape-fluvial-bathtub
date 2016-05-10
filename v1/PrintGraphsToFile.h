//
//  PrintGraphsToFile.h
//  VaryingFloodDepth
//
//  Created by a1091793 on 1/10/2015.
//  Copyright © 2015 University of Adelaide. All rights reserved.
//

#ifndef PrintGraphsToFile_h
#define PrintGraphsToFile_h

#include "Types.h"
#include <boost/graph/graphviz.hpp>
#include <boost/graph/graphml.hpp>


void printGraphsToFile(Graph grph, std::string file_name_prefix)
{
    /********************************************/
    /*           Print graphs to file           */
    /********************************************/
    boost::dynamic_properties dp;
    dp.property("node_id", boost::get(&ChannelNode::node_id, grph));
    dp.property("row", boost::get(&ChannelNode::row, grph));
    dp.property("col", boost::get(&ChannelNode::col, grph));
    dp.property("down_cntrol_id", boost::get(&ChannelNode::down_cntrl_id, grph));
    dp.property("down_cntrl_dist", boost::get(&ChannelNode::down_cntrl_dist, grph));
    dp.property("up_cntrl_id", boost::get(&ChannelNode::up_cntrl_id, grph));
    dp.property("up_cntrl_dist", boost::get(&ChannelNode::up_cntrl_dist, grph));
    dp.property("control_node_id", boost::get(&ChannelNode::cntrl_node_id, grph));
    dp.property("type", boost::get(&ChannelNode::type, grph));
    dp.property("terminal_type", boost::get(&ChannelNode::terminal_type, grph));
    dp.property("level", boost::get(&ChannelNode::level, grph));
	dp.property("inundation", boost::get(&ChannelNode::inundation, grph));
    dp.property("elevation", boost::get(&ChannelNode::elevation, grph));
    dp.property("distance", boost::get(&ChannelLink::distance, grph));
	
    
    std::string dot_prefix = file_name_prefix + ".dot";
    std::ofstream fs(dot_prefix.c_str());
    if (fs.is_open())
    {
        boost::write_graphviz_dp(fs, grph, dp);
        fs.close();
    }
    
    VertexDescMap idxMap2;
    boost::associative_property_map<VertexDescMap> indexMap2(idxMap2);
    VertexIterator di2, dj2;
    boost::tie(di2, dj2) = boost::vertices(grph);
    for(int i = 0; di2 != dj2; ++di2,++i)
    {
        boost::put(indexMap2, (*di2), grph[*di2].node_id);
    }
    std::string ml_prefix = file_name_prefix + ".graphml";
    std::ofstream fs2(ml_prefix.c_str());
    if (fs2.is_open())
    {
        boost::write_graphml(fs2, grph, indexMap2, dp);
        fs2.close();
    }
//    if (fs2.is_open())
//    {
//        boost::write_graphml(fs2, grph, dp);
//        fs2.close();
//    }
}

//std::cout << "\n\n*************************************\n";
//std::cout <<     "*     Printing Controls Graph       *\n";
//std::cout <<     "*************************************" << std::endl;
//std::string controls_file_name = "control_graph";
//printGraphsToFile(controls_grph, controls_file_name, true);
//
//boost::dynamic_properties dp2;
//dp2.property("node_id", boost::get(&ChannelNode::node_id, controls_grph));
//dp2.property("row", boost::get(&ChannelNode::row, controls_grph));
//dp2.property("col", boost::get(&ChannelNode::col, controls_grph));
//dp2.property("down_cntrol_id", boost::get(&ChannelNode::up_cntrl_id, controls_grph));
//dp2.property("down_cntrl_dist", boost::get(&ChannelNode::down_cntrl_dist, controls_grph));
//dp2.property("up_cntrl_id", boost::get(&ChannelNode::up_cntrl_id, controls_grph));
//dp2.property("up_cntrl_dist", boost::get(&ChannelNode::up_cntrl_dist, controls_grph));
//dp2.property("type", boost::get(&ChannelNode::type, controls_grph));
//dp2.property("level", boost::get(&ChannelNode::level, controls_grph));
//std::ofstream fs5("control_graph.dot");
//if (fs5.is_open())
//{
//    boost::write_graphviz_dp(fs5, controls_grph, dp2);
//}
//std::cout << "\n\n*************************************\n";
//std::cout <<     "*    Printing Graph to .graphml     *\n";
//std::cout <<     "*************************************" << std::endl;
//VertexDescMap idxMap2;
//boost::associative_property_map<VertexDescMap> indexMap2(idxMap2);
//VertexIterator di2, dj2;
//boost::tie(di2, dj2) = boost::vertices(controls_grph);
//for(int i = 0; di2 != dj2; ++di2,++i)
//{
//    boost::put(indexMap2, (*di2), channel_grph[*di2].node_id);
//}
//std::ofstream fs6("control_graph.graphml");
//if (fs6.is_open())
//{
//    boost::write_graphml(fs6, controls_grph, indexMap2, dp2);
//}


#endif /* PrintGraphsToFile_h */
