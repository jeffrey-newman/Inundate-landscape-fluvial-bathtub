cmake_minimum_required(VERSION 3.1)
project(inundate-landscape)
set (CMAKE_CXX_STANDARD 11)


find_package(Boost REQUIRED COMPONENTS filesystem program_options graph system)
find_package(GDAL)

set(AHHZ_RASTER_UTIL "Not Specified" CACHE PATH "Path to Alex Hagen-Zanker's Raster Utility Include direction (select above blink)")
set(AHHZ_RASTER_ITERATOR "Not Specified" CACHE PATH "Path to Alex Hagen-Zanker's Raster Iterator Include direction (select folder above blink)")

include_directories(${GDAL_INCLUDE_DIR} )
include_directories(${Boost_INCLUDE_DIRS} )
include_directories(${AHHZ_RASTER_UTIL} )
include_directories(${AHHZ_RASTER_ITERATOR} )

set( exe_srcs
        Main.cpp
        InundateLandscape.h
)

set( lib_sources
		InundateLandscape.cpp
		Neighbourhood.cpp
		Neighbourhood_Impl.cpp
        GraphSearches.cpp
        IsChannel.cpp
        PrintGraphsToFile.cpp
        RasterCoordinates.cpp
 )

set ( lib_headers
		GraphSearches.h
		IsChannel.h
		PrintGraphsToFile.h
		RasterCoordinates.h
		ReadGraphsFromFile.h
		ReadInControlsAndGuages.h
		remove_processed.h
		Types.h
		Neighbourhood.h
        InundateLandscape.h
	)

include_directories(eigen)

file(GLOB_RECURSE AHHZ_Utils
"${AHHZ_RASTER_UTIL}/*.h"
"${AHHZ_RASTER_UTIL}/*.hpp"
"${AHHZ_RASTER_UTIL}/*.cpp"
"${AHHZ_RASTER_ITERATOR}/*.h"
"${AHHZ_RASTER_ITERATOR}/*.hpp"
"${AHHZ_RASTER_ITERATOR}/*.cpp"
)


add_library(inundate "${lib_sources}" "${lib_headers}" "${AHHZ_Utils}")
target_link_libraries(inundate ${GDAL_LIBRARY} ${Boost_LIBRARIES})

add_executable(INUNDATE "${exe_srcs}" )
target_link_libraries(INUNDATE inundate)