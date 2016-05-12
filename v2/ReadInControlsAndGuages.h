//
//  ReadInControls.h
//  VaryingFloodDepth
//
//  Created by a1091793 on 23/09/2015.
//  Copyright © 2015 University of Adelaide. All rights reserved.
//

#ifndef ReadInControls_h
#define ReadInControls_h

#include <iostream>
#include <fstream>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/smart_ptr.hpp>
#include "Types.h"

#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/phoenix_core.hpp>
#include <boost/spirit/include/phoenix_operator.hpp>
#include <boost/spirit/include/phoenix_object.hpp>
#include <boost/fusion/include/adapt_struct.hpp>
#include <boost/fusion/include/io.hpp>


namespace qi = boost::spirit::qi;



BOOST_FUSION_ADAPT_STRUCT(
                        GuageControl,
                          (double, x_coord)
                          (double, y_coord)
                          (double, level)
                          )

template <typename Iterator>
struct GuageParser : qi::grammar<Iterator, GuageControl(), qi::space_type>
{
    GuageParser(): GuageParser::base_type(start)
    {
        start %= qi::double_ >> qi::double_ >> qi::double_;
    }
    qi::rule<Iterator, GuageControl(), qi::space_type> start;
};


boost::shared_ptr<std::vector<GuageControl> >
readInGuages(const boost::filesystem::path & guage_file)
{
    boost::shared_ptr<std::vector<GuageControl> > guages( new std::vector<GuageControl>);
    if (boost::filesystem::exists(guage_file))
    {
        std::ifstream fs(guage_file.c_str());
        if (fs.is_open())
        {
            GuageParser<std::string::const_iterator> g; // Our grammar
            std::string str;
            getline(fs, str); // Header line.
            while (getline(fs, str))
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
                        guages->push_back(guage);
                    }
                    else
                    {
                        std::cout << "-------------------------\n";
                        std::cout << "Parsing guage file " + guage_file.string() + " failed:\n";
                        std::cout << " At: " << str << "\n";
                        std::cout << "-------------------------\n";
                    }
                }
            }
        }
        else
        {
            std::string msg = "Could not open guage file " + guage_file.string();
            throw std::runtime_error(msg);
            
        }
    }
    else
    {
        std::string msg = "Guage file does not exist: " + guage_file.string();
        throw std::runtime_error(msg);
    }
    return (guages);

}



BOOST_FUSION_ADAPT_STRUCT(
                          ChannelNode,
                          (int, node_id)
                          //(int, row)
                          //(int, col)
                          (int, type)
                          (double, level)
                          )

template <typename Iterator>
struct ControlParser : qi::grammar<Iterator, ChannelNode(), qi::space_type>
{
    ControlParser() : ControlParser::base_type(start)
    {
        start %=
        qi::int_
        //>>  qi::int_
        //>>  qi::int_
        >>  qi::int_
        >>  qi::double_
        ;
    }
    
    qi::rule<Iterator, ChannelNode(), qi::space_type> start;
};


boost::shared_ptr<std::vector<ChannelNode> >
readInControls(const boost::filesystem::path & controls_file)
{
    boost::shared_ptr<std::vector<ChannelNode> > controls( new std::vector<ChannelNode>);
    if (boost::filesystem::exists(controls_file))
    {
        std::ifstream fs(controls_file.c_str());
        if (fs.is_open())
        {
            ControlParser<std::string::const_iterator> g; // Our grammar
            std::string str;
            getline(fs, str); // Header line.
            while (getline(fs, str))
            {
                if (!(str.empty() || str[0] == '#'))
                {
                    ChannelNode control;
                    std::string::const_iterator iter = str.begin();
                    std::string::const_iterator end = str.end();
                    bool r = phrase_parse(iter, end, g, qi::space, control);
                    
                    if (r && iter == end)
                    {
                        //Place the guage into the vector
                        controls->push_back(control);
                    }
                    else
                    {
                        std::cout << "-------------------------\n";
                        std::cout << "Parsing controls file " + controls_file.string() + " failed:\n";
                        std::cout << " At: " << str << "\n";
                        std::cout << "-------------------------\n";
                    }
                }
            }
        }
        else
        {
            std::string msg = "Could not open controls file " + controls_file.string();
            throw std::runtime_error(msg);
        }
    }
    else
    {
        std::string msg = "Controls file does not exist: " + controls_file.string();
        throw std::runtime_error(msg);
    }
    
}

void
writeControls(const boost::filesystem::path & file, Graph & channel_grph)
{
    /********************************************/
    /*       Print table of control points      */
    /********************************************/
    std::cout << "\n\n*************************************\n";
    std::cout << "*  Writing table of control points  *\n";
    std::cout << "*************************************" << std::endl;
    // Iterate through the vertices and print them out
    std::ofstream fs(file.c_str());
    if (fs.is_open())
    {
        fs << "# id\trow\tcol\ttype\tlevel\n";
        fs << "# Control nodes:\n";
        std::pair<VertexIterator, VertexIterator> vp;
        for (vp = boost::vertices(channel_grph); vp.first != vp.second; ++vp.first)
        {
            if (channel_grph[*(vp.first)].type > 1)
            {
                fs << channel_grph[*(vp.first)].node_id << "\t" << channel_grph[*(vp.first)].row << "\t" << channel_grph[*(vp.first)].col << "\t" << channel_grph[*(vp.first)].type << "\t" <<
                channel_grph[*(vp.first)].level << std::endl;
            }
        }
        fs << "# Non control nodes:\n";
        for (vp = boost::vertices(channel_grph); vp.first != vp.second; ++vp.first)
        {
            if (channel_grph[*(vp.first)].type == 1)
            {
                fs << channel_grph[*(vp.first)].node_id << "\t" << channel_grph[*(vp.first)].row << "\t" << channel_grph[*(vp.first)].col << "\t" << channel_grph[*(vp.first)].type << "\t" <<
                channel_grph[*(vp.first)].level << std::endl;
            }
        }
        fs.close();
    }
}



//template <typename Iterator>
//bool parse_dir_name(Iterator first, Iterator last, std::string const & text2parse, std::pair<int, std::string> & return_val)
//{
//    
//    qi::rule<Iterator, std::string(), qi::space_type> name_parse = qi::lexeme[ +qi::char_];
//    
//    bool r = qi::phrase_parse(first, last,
//                              
//                              //  Begin grammar
//                              (
//                               qi::lit(dir_prefix) >> qi::int_[phoenix::ref(return_val.first) = qi::_1]
//                               >> '-'
//                               >> name_parse[phoenix::ref(return_val.second) = qi::_1]
//                               ),
//                              //  End grammar
//                              qi::space);
//    if (!r || first != last) // fail if we did not get a full match
//        return (false);
//    return (r);
//}

#endif /* ReadInControls_h */


