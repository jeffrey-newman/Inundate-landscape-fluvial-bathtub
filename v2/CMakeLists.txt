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

set( sources
		InundateLandscape.cpp
		Main.cpp
		Neighbourhood.cpp
		Neighbourhood_Impl.cpp
 )

set ( headers
		GraphSearches.h
		IsChannel.h
		PrintGraphsToFile.h
		RasterCoordinates.h
		ReadGraphsFromFile.h
		ReadInControlsAndGuages.h
		remove_processed.h
		Types.h
		Neighbourhood.h
	)

include_directories(eigen)

add_executable(inundate "${sources}" "${headers}")
target_link_libraries(inundate ${GDAL_LIBRARY} ${Boost_LIBRARIES})