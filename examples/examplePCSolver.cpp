// Standard C++ libraries
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <queue>

// Boost libraries
// #include <boost/shared_ptr.hpp>
#include <boost/property_map/dynamic_property_map.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphml.hpp>
#include <boost/function.hpp>
#include <boost/program_options.hpp>

// OpenCV libraries
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Custom header files
#include "CMAPF/PCSolver.hpp"

using namespace boost;
using namespace CMAPF;


int main(int argc, char *argv[])
{
	Pair edge_array[4] = { Pair(0,1), Pair(1,2),
													Pair(3, 4), Pair(4, 5) };
		
	PrecedenceConstraintGraph G(6);

	property_map<PrecedenceConstraintGraph, meta_data_t>::type name = get(meta_data_t(), G);
		
	float eps = 0.0625;

	name[0] = meta_data (std::make_pair(eps*5, eps*2), std::make_pair(eps*6, eps*5), std::vector <int> (1, 0), 0, 4, 0);
	name[1] = meta_data (std::make_pair(eps*6, eps*5), std::make_pair(eps*6, eps*6), std::vector <int> (1, 0), 4, 5, 0);
	name[2] = meta_data (std::make_pair(eps*6, eps*6), std::make_pair(eps*6, eps*7), std::vector <int> (1, 0), 5, 6, 2);

	name[3] = meta_data (std::make_pair(eps*5, eps*1), std::make_pair(eps*6, eps*3), std::vector <int> (1, 1), 0, 3, 0);
	name[4] = meta_data (std::make_pair(eps*6, eps*3), std::make_pair(eps*6, eps*4), std::vector <int> (1, 1), 3, 4, 0);
	name[5] = meta_data (std::make_pair(eps*6, eps*4), std::make_pair(eps*6, eps*8), std::vector <int> (1, 1), 4, 8, 0);

  	for (int i = 0; i < 4; ++i)
		add_edge(edge_array[i].first, edge_array[i].second, G);

	PCSolver p;
	p.ICTS(G, 1);
	// std::cout << count << std::endl;
	
	return 0;
}

// int main(int argc, char *argv[])
// {
//   Pair edge_array[8] = { Pair(0,1), Pair(1,2),
//                           Pair(3, 4), Pair(4, 5),
//                           Pair(6, 7), Pair(7, 8),
//                           Pair(9, 10), Pair(10, 11) };
    
//   PrecedenceConstraintGraph G(12);

//   property_map<PrecedenceConstraintGraph, meta_data_t>::type name = get(meta_data_t(), G);
    
//   float eps = 0.0625;

//   name[0] = meta_data (std::make_pair(eps*5, eps*2), std::make_pair(eps*6, eps*9), std::vector <int> (1, 0), 0, 8, 0);
//   name[1] = meta_data (std::make_pair(eps*6, eps*9), std::make_pair(eps*6, eps*10), std::vector <int> (1, 0), 8, 9, 0);
//   name[2] = meta_data (std::make_pair(eps*6, eps*10), std::make_pair(eps*6, eps*11), std::vector <int> (1, 0), 9, 10, 6);

//   name[3] = meta_data (std::make_pair(eps*5, eps*1), std::make_pair(eps*6, eps*7), std::vector <int> (1, 1), 0, 7, 0);
//   name[4] = meta_data (std::make_pair(eps*6, eps*7), std::make_pair(eps*6, eps*8), std::vector <int> (1, 1), 7, 8, 0);
//   name[5] = meta_data (std::make_pair(eps*6, eps*8), std::make_pair(eps*6, eps*12), std::vector <int> (1, 1), 8, 12, 4);

//   name[6] = meta_data (std::make_pair(eps*8, eps*1), std::make_pair(eps*5, eps*5), std::vector <int> (1, 2), 0, 6, 0);
//   name[7] = meta_data (std::make_pair(eps*5, eps*5), std::make_pair(eps*5, eps*6), std::vector <int> (1, 2), 6, 7, 0);
//   name[8] = meta_data (std::make_pair(eps*5, eps*6), std::make_pair(eps*5, eps*13), std::vector <int> (1, 2), 7, 14, 2);

//   name[9] = meta_data (std::make_pair(eps*8, eps*4), std::make_pair(eps*5, eps*3), std::vector <int> (1, 3), 0, 5, 0);
//   name[10] = meta_data (std::make_pair(eps*5, eps*3), std::make_pair(eps*5, eps*4), std::vector <int> (1, 3), 5, 6, 0);
//   name[11] = meta_data (std::make_pair(eps*5, eps*4), std::make_pair(eps*5, eps*14), std::vector <int> (1, 2), 6, 16, 0);

//   for (int i = 0; i < 8; ++i)
//     add_edge(edge_array[i].first, edge_array[i].second, G);

//   PCSolver p;
//   p.ICTS(G, 1);

//   return 0;
//   // std::cout << count << std::endl;
// }