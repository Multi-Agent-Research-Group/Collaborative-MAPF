// Standard C++ libraries
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <queue>
#include <random>

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
#include "../include/utils/graphFileUtils.hpp"
// #include "../include/utils/LoadGraphfromFile.hpp"

int INF = 100000;

namespace po = boost::program_options;

// using namespace BGL_DEFINITIONS;
using namespace cv;

std::vector<std::vector<bool>> map;

bool evaluateIndividualConfig(Eigen::VectorXd config)
{
	if(map[(int)config[0]][(int)config[1]])
		return true;
	return false;
}

bool evaluateIndividualEdge(Graph &graph, Edge& e) // returns false if in collision
{
	graph[e].isEvaluated = true;

	Vertex source_vertex = source(e, graph);
	Vertex target_vertex = target(e, graph);

	Eigen::VectorXd sourceState(2);
	sourceState << graph[source_vertex].state;

	Eigen::VectorXd targetState(2);
	targetState << graph[target_vertex].state;

	// std::cout<<"nStates:"<<nStates<<std::endl;

	if (!evaluateIndividualConfig(sourceState) || !evaluateIndividualConfig(targetState))
	{
		graph[target_vertex].status = CollisionStatus::BLOCKED;
		graph[e].status = CollisionStatus::BLOCKED;
		graph[e].length = INF;
		return false;
	}

	return true;
}

void preprocess_graph(Graph &g, std::string output_filepath)
{
	int mNumVertices = boost::num_vertices(g);
	std::vector<std::vector<int>> edgeWeights(mNumVertices, std::vector<int>(mNumVertices, INF));
	int mUnitEdgeLength = 1;
	VertexIter vi_1, viend_1;
	VertexIter vi_2, viend_2;

	for(int i=0; i<mNumVertices; i++)
		edgeWeights[i][i] = 0;


	int edge = 0;
	for (boost::tie(vi_1, viend_1) = vertices(g); vi_1 != viend_1; ++vi_1) 
	{
		Vertex vertex_1 = *vi_1;
		OutEdgeIter ei, ei_end;
		for (boost::tie(ei, ei_end) = out_edges(vertex_1, g); ei != ei_end; ++ei) 
		{
			Vertex vertex_2 = target(*ei, g);
			if(vertex_1==vertex_2){
				std::cout << "ehy\n";
				continue;
			}
			Edge e = *ei;
			if(!g[e].isEvaluated){
				if(evaluateIndividualEdge(g,e)){
					edge += 1;
					edgeWeights[vertex_1][vertex_2] = mUnitEdgeLength;
					edgeWeights[vertex_2][vertex_1] = mUnitEdgeLength;
				}
			}
		}
	}
	// std::cout << edge << std::endl;
	// std::cout << "DONE\n";
	// std::cin.get();

	int count = 0;

	VertexIter vi_3, viend_3;
	for (boost::tie(vi_1, viend_1) = vertices(g); vi_1 != viend_1; ++vi_1){
		count++;
		std::cout << count << std::endl;
		for (boost::tie(vi_2, viend_2) = vertices(g); vi_2 != viend_2; ++vi_2){ 
			for (boost::tie(vi_3, viend_3) = vertices(g); vi_3 != viend_3; ++vi_3) 
			{
				Vertex vertex_1 = *vi_1;
				Vertex vertex_2 = *vi_2;
				Vertex vertex_3 = *vi_3;
				if (edgeWeights[vertex_2][vertex_3] + 0.00001 > 
					(edgeWeights[vertex_2][vertex_1] + edgeWeights[vertex_1][vertex_3])
					&& (edgeWeights[vertex_2][vertex_1] != INF && 
						edgeWeights[vertex_1][vertex_3] != INF))
						edgeWeights[vertex_2][vertex_3] = 
							edgeWeights[vertex_2][vertex_1] + edgeWeights[vertex_1][vertex_3];
			}
		}
	}

	std::ofstream fout;
	fout.open(output_filepath);
	for (boost::tie(vi_1, viend_1) = vertices(g); vi_1 != viend_1; ++vi_1) 
	{
		for (boost::tie(vi_2, viend_2) = vertices(g); vi_2 != viend_2; ++vi_2) 
		{
			Vertex vertex_1 = *vi_1;
			Vertex vertex_2 = *vi_2;
			fout << vertex_1 << " " << vertex_2 << " " << edgeWeights[vertex_1][vertex_2] << std::endl;
		}
	}
	fout.close();
}

std::vector<std::vector<bool>> extract_map(const std::string map_fname)
{
    std::vector<std::vector<bool>> extracted_map;
    std::ifstream ifs(map_fname);
    std::string line;
    int height, width;
    
    std::getline(ifs,line); // Assuming map type is octile, so skip
    
    std::getline(ifs,line);
    int i = 0;
    for (; i < line.length(); i++) if (std::isdigit(line[i])) break;
    line = line.substr(i,line.length() - i);
    height = std::atoi(line.c_str());
    std::cout << "Height: " << height;

    std::getline(ifs,line);    
    for (i = 0; i < line.length(); i++) if (std::isdigit(line[i])) break;
    line = line.substr(i,line.length() - i);
    width = std::atoi(line.c_str());
    std::cout << " Width: " << width << std::endl;

    std::getline(ifs,line); // This is the line that says "map"

    for (i = 0; i < height; i++)
    {
        std::vector<bool> map_line;
        std::getline(ifs,line);
        for (int j = 0; j < width; j++)
        {
            map_line.push_back(line[j] == '.');
            // std::cout << map_line.back() << " ";
        }
        std::cout << std::endl;

        extracted_map.push_back(map_line);
    }

    return extracted_map;
}

int main(int argc, char* argv[]){
	srand(time(NULL));
    po::options_description desc("Select Map File & Output Filename for Graph");
    desc.add_options()
        ("help", "produce help message")
        ("map_file,m",po::value<std::string>()->required(), "path to map file")
        ("graph_file,g",po::value<std::string>()->required(), "path to map file")
        ("output_file,o",po::value<std::string>()->required(), " path to graphml file")
    ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc,argv,desc), vm);

    if(vm.count("help")) {
        std::cout<<desc<<std::endl;
        return 1;
    }
    
    std::string mRoadmapFileName=vm["graph_file"].as<std::string>();
    std::string map_filepath=vm["map_file"].as<std::string>();
    std::string output_filepath=vm["output_file"].as<std::string>();

    map = extract_map(map_filepath);
	//Pre-Processing
	Graph mGraph;
	create_vertices(mGraph,get(&VProp::state,mGraph),mRoadmapFileName,2,get(&EProp::prior,mGraph));
	create_edges(mGraph,get(&EProp::length,mGraph));
	preprocess_graph(mGraph, output_filepath);
}

