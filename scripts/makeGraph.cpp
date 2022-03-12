#include "../include/utils/graphFileUtils.hpp"
#include <fstream>
#include <chrono>
#include <time.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Boost libraries
// #include <boost/shared_ptr.hpp>
#include <boost/property_map/dynamic_property_map.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphml.hpp>
#include <boost/function.hpp>
#include <boost/program_options.hpp>
#include <boost/lexical_cast.hpp>
#include "opencv2/imgproc/imgproc_c.h"

#include <Eigen/Dense>

namespace po = boost::program_options;

int mHeight;
int mWidth;

template<typename T>
struct matrix_hash : std::unary_function<T, size_t> {
  std::size_t operator()(T const& matrix) const {
    // Note that it is oblivious to the storage order of Eigen matrix (column- or
    // row-major). It will give you the same hash value for two different matrices if they
    // are the transpose of each other in different storage order.
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i) {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

void displayGraphOnImage(Graph graph, cv::Mat mImage, int scale)
{
    cv::Mat image;
    cv::cvtColor(mImage, image, CV_GRAY2BGR);
    // image = cv::Scalar(255,255,255);
    int numberOfRows = image.rows;
    int numberOfColumns = image.cols;

    EdgeIter ei, ei_end;
    for(boost::tie(ei,ei_end) = edges(graph); ei!=ei_end;++ei)
    {
        cv::Point source_Point((int)(graph[source(*ei,graph)].state[0]*scale), 
            (int)(numberOfColumns-graph[source(*ei,graph)].state[1]*scale) );
        cv::Point target_Point((int)(graph[target(*ei,graph)].state[0]*scale), 
            (int)(numberOfRows-graph[target(*ei,graph)].state[1]*scale) );
        cv::line(image, source_Point, target_Point, cv::Scalar(0, 255, 255), scale/2);
    }

    VertexIter vi, vi_end;
    for (boost::tie(vi, vi_end) = vertices(graph); vi != vi_end; ++vi)
    {
        double x_point = graph[*vi].state[0]*scale;
        double y_point = numberOfRows - graph[*vi].state[1]*scale;
        cv::Point centre_Point((int)x_point, (int)y_point);
        cv::circle(image, centre_Point, scale/2,  cv::Scalar(0, 150, 0), -1);
    }

    cv::namedWindow("Graph Visualization",cv::WINDOW_NORMAL);
    cv::imshow("Graph Visualization", image);
    cv::waitKey(10000);
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
        }

        extracted_map.push_back(map_line);
    }

    return extracted_map;
}

Graph buildGraph(std::vector <std::vector <bool>> map){
    Graph graph;
    std::unordered_map<Eigen::VectorXd, Vertex, matrix_hash<Eigen::VectorXd>> configToNodeMap;
    for(int x_coord = 0; x_coord < map.size(); x_coord += 1){
        for(int y_coord = 0; y_coord < map[x_coord].size(); y_coord += 1){
            Eigen::VectorXd config(2);
            config << x_coord,y_coord;
            Vertex new_node;
            new_node = add_vertex(graph);
            graph[new_node].state = config;
            graph[new_node].vertex_index = boost::num_vertices(graph) - 1;
            configToNodeMap[config]=new_node;
        }
    }   

    for(int x_coord = 0; x_coord < map.size(); x_coord += 1){
        for(int y_coord = 0; y_coord < map[x_coord].size(); y_coord += 1){
            Eigen::VectorXd config(2);
            config << x_coord, y_coord;
        
            if(x_coord != map.size()-1)
            {
                Eigen::VectorXd right_config(2);
                right_config << x_coord + 1, y_coord;

                std::pair<Edge,bool> curEdge = 
                    boost::add_edge(configToNodeMap[config], configToNodeMap[right_config], graph);
            }

            if(y_coord != map[x_coord].size()-1)
            {
                Eigen::VectorXd down_config(2);
                down_config << x_coord, y_coord + 1;

                std::pair<Edge,bool> curEdge = 
                    boost::add_edge(configToNodeMap[config], configToNodeMap[down_config], graph);
            }   
        }
    }
    return graph;
}


int main(int argc, char* argv[])
{
    srand(time(NULL));
    po::options_description desc("Select Map File & Output Filename for Graph");
    desc.add_options()
        ("help", "produce help message")
        ("map_file,m",po::value<std::string>()->required(), "path to map file")
        ("output_graph,g",po::value<std::string>()->required(), " path to graphml file")
    ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc,argv,desc), vm);

    if(vm.count("help")) {
        std::cout<<desc<<std::endl;
        return 1;
    }
    
    std::string map_filepath=vm["map_file"].as<std::string>();
    std::string output_filepath=vm["output_graph"].as<std::string>();
    
    // Space Information
    std::vector <std::vector <bool>> map = extract_map(map_filepath);
    Graph graph = buildGraph(map);

    std::cout<<"#Vertices: "<<boost::num_vertices(graph)<<
        "#Edges: "<<boost::num_edges(graph)<<std::endl;

    cv::Mat image = cv::imread("../data/obstacles/room.png", 0);
    displayGraphOnImage(graph, image, 10);

    write_graph(graph,output_filepath,2);
    
    return 0;
}