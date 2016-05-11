//
//  Types.h
//  move_creek
//
//  Created by a1091793 on 23/01/2015.
//  Copyright (c) 2015 University of Adelaide. All rights reserved.
//

#ifndef Types_h
#define Types_h

#include <vector>
#include <tuple>
#include <boost/shared_ptr.hpp>
#include <boost/graph/adjacency_list.hpp>




//enum TNodeType{NOT_CNTRL = 1, OUTFLW_CNTRL = 2, GUAGE_CNTRL = 4, JUNCT_CNTRL = 8, SOURCE_CNTRL = 16};
enum TNodeType{NOT_CNTRL = 1, TERMINAL_CNTRL = 2, GUAGE_CNTRL = 4, JUNCT_CNTRL = 8};
enum TerminalType{NOT_TERMINAL = 1, OUTFLOW = 2, SOURCE=3};

struct ChannelNode;
struct ChannelLink;
//namespace graph = boost::graph;
//typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, ChannelNode > Graph;
typedef boost::adjacency_list<boost::setS, boost::setS, boost::undirectedS, ChannelNode, ChannelLink > Graph;
typedef boost::graph_traits<Graph>::vertex_descriptor VertexDescriptor;
typedef boost::graph_traits<Graph>::edge_descriptor EdgeDescriptor;
typedef boost::graph_traits<Graph>::out_edge_iterator OutEdgeIterator;
typedef boost::graph_traits<Graph>::vertex_iterator VertexIterator;
typedef boost::graph_traits<Graph>::edge_iterator EdgeIterator;
typedef std::map<VertexDescriptor, size_t> VertexDescMap;
typedef std::map<int, VertexDescriptor> VertexIDMap;
typedef boost::graph_traits<Graph>::degree_size_type OutDegreeType;

struct ChannelNode
{
    int node_id;
    int cntrl_node_id;
    int row;
    int col;
    int down_cntrl_id;
    int down_cntrl_dist;
    int up_cntrl_id;
    int up_cntrl_dist;
    int type;
    int terminal_type;
    double level;
    double elevation;
	double inundation;
    bool processed;
    
    ChannelNode
    (): node_id(-1), cntrl_node_id(-1), row(-1), col(-1), down_cntrl_id(-1), down_cntrl_dist(-1), up_cntrl_id(-1), up_cntrl_dist(-1), type(NOT_CNTRL), terminal_type(NOT_TERMINAL), level(-999), elevation(-999), inundation(-999), processed(false)
    {
        
    }
};

struct ChannelLink
{
    int link_id;
    int distance;
    
    ChannelLink(): link_id(-1), distance(-1)
    {
        
    }
};


typedef std::vector<ChannelNode> Set;

struct GuageControl
{
    double x_coord;
    double y_coord;
    double level;
    
    GuageControl() : x_coord(-1), y_coord(-1), level(-999)
    {
        
    }
};


const int NON_FEATURE = 0;


#endif
