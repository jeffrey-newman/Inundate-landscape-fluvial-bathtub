//
//  ReadInControls.h
//  VaryingFloodDepth
//
//  Created by a1091793 on 23/09/2015.
//  Copyright © 2015 University of Adelaide. All rights reserved.
//

#ifndef ReadInControls_h
#define ReadInControls_h

#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/phoenix_core.hpp>
#include <boost/spirit/include/phoenix_operator.hpp>
#include <boost/spirit/include/phoenix_object.hpp>
#include <boost/fusion/include/adapt_struct.hpp>
#include <boost/fusion/include/io.hpp>

namespace qi = boost::spirit::qi;

struct GuageControl
{
    double x_coord;
    double y_coord;
    double level;
    
    GuageControl() : x_coord(-1), y_coord(-1), level(-999)
    {
        
    }
};

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


