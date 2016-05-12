//
//  IsNotChannel.h
//  Inundate Landscape
//
//  Created by a1091793 on 7/10/2015.
//  Copyright © 2015 University of Adelaide. All rights reserved.
//

#ifndef IsNotChannel_h
#define IsNotChannel_h

#include "Types.h"

std::pair<bool, VertexDescriptor> isCreek(int row, int col, std::map<int, std::map<int, VertexDescriptor>  > &  channel_pixels);

#endif /* IsNotChannel_h */
