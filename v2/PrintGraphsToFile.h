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


void printGraphsToFile(Graph grph, std::string file_name_prefix);


#endif /* PrintGraphsToFile_h */
