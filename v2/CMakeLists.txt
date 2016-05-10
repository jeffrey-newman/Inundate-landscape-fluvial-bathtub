cmake_minimum_required(VERSION 3.1)
project(inundate-landscape)
set (CMAKE_CXX_STANDARD 11)


find_package(Boost REQUIRED COMPONENTS filesystem program_options graph system)
find_package(GDAL)

include_directories(${GDAL_INCLUDE_DIR})
include_directories(${Boost_INCLUDE_DIRS} )

set( sources
		InundateLandscape.cpp
		Map_Matrix_Impl.cpp
		Map_Matrix.cpp
		Neighbourhood_Impl.cpp
		Neighbourhood.cpp
		Print_Impl.cpp
		Print.cpp
		ReadInMap_Impl.cpp
		ReadInMap.cpp
 )

set ( headers
		GraphSearches.h
		IsChannel.h
		Map_Matrix.h
		Print.h
		Neighbourhood.h
		PrintGraphsToFile.h
		RasterCoordinates.h
		ReadGraphsFromFile.h
		ReadInControls.h
		ReadInMap.h
		remove_processed.h
		Types.h
	)

include_directories(eigen)

add_executable(inundate "${sources}" "${headers}")
target_link_libraries(inundate ${GDAL_LIBRARY} ${Boost_LIBRARIES})