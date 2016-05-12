//
//  IsChannel.cpp
//  inundate-landscape
//
//  Created by a1091793 on 12/05/2016.
//
//

#include <stdio.h>
#include "IsChannel.h"

std::pair<bool, VertexDescriptor> isCreek(int row, int col, std::map<int, std::map<int, VertexDescriptor>  > &  channel_pixels)
{
    typedef std::map<int, std::map<int, VertexDescriptor>  >::iterator RowIt;
    typedef std::map<int, VertexDescriptor>::iterator ColIt;
    RowIt r_it = channel_pixels.find(row);
    if (r_it != channel_pixels.end())
    {
        std::map<int, VertexDescriptor> & col_map  = r_it->second;
        ColIt c_it = col_map.find(col);
        if (c_it != col_map.end())
        {
            return (std::make_pair(true, c_it->second));
        }
        else
        {
            return (std::make_pair(false, channel_pixels.begin()->second.begin()->second));
        }
    }
    return (std::make_pair(false, channel_pixels.begin()->second.begin()->second));
}