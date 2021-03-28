#include "../include/utils/graphFileUtils.hpp"
#include <fstream>
#include <chrono>
#include <boost/lexical_cast.hpp>
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

std::istream & operator >>
(std::istream & s,
 Eigen::VectorXd & m)
{
    for (int i = 0; i < m.size(); ++i)
      s >> m(i);
  return s;
}

namespace std
{
template<> struct less<Eigen::VectorXd>
{
	bool operator() (Eigen::VectorXd const& a, Eigen::VectorXd const& b) const
	{
		assert(a.size()==b.size());
		for(size_t i=0;i<a.size();++i)
		{
			if(a[i]<b[i]) return true;
			if(a[i]>b[i]) return false;
		}
		return false;
	}
};
}

namespace po = boost::program_options;

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

void displayGraphOnImage(Graph graph, cv::Mat mImage)
{
	cv::Mat image;
	cv::cvtColor(mImage, image, CV_GRAY2BGR);
	image = cv::Scalar(255,255,255);
	int numberOfRows = image.rows;
	int numberOfColumns = image.cols;

	EdgeIter ei, ei_end;
	for(boost::tie(ei,ei_end) = edges(graph); ei!=ei_end;++ei)
	{
		cv::Point source_Point((int)(graph[source(*ei,graph)].state[0]*numberOfColumns), 
			(int)((1-graph[source(*ei,graph)].state[1])*numberOfColumns));
		cv::Point target_Point((int)(graph[target(*ei,graph)].state[0]*numberOfColumns), 
			(int)((1-graph[target(*ei,graph)].state[1])*numberOfColumns));
		cv::line(image, source_Point, target_Point, cv::Scalar(0, 255, 255), 2);
	}

	VertexIter vi, vi_end;
	for (boost::tie(vi, vi_end) = vertices(graph); vi != vi_end; ++vi)
	{
		double x_point = graph[*vi].state[0]*numberOfColumns;
		double y_point = (1 - graph[*vi].state[1])*numberOfRows;
		cv::Point centre_Point((int)x_point, (int)y_point);
		cv::circle(image, centre_Point, 4,  cv::Scalar(0, 150, 0), -1);
	}

	cv::namedWindow("Graph Visualization",cv::WINDOW_NORMAL);
	cv::imshow("Graph Visualization", image);
	cv::waitKey(100);
}

bool isEqual(double a, double b)
{
	return fabs(a - b) < 0.0000001;
}

bool isFringeNode(Eigen::VectorXd config, double epsilon)
{
	if(config[0]+epsilon>0.999999999)
		return true;
	if(config[0]-epsilon<0.000000001)
		return true;
	if(config[1]+epsilon>0.999999999)
		return true;
	if(config[1]-epsilon<0.000000001)
		return true;
	return false;
}

void drop_vertices(Graph &graph, double epsilon)
{
	int select_vertex = rand()%int(boost::num_vertices(graph));
	int count=0;

	Vertex v = vertex(select_vertex, graph);

	// std::cout<<graph[v].vertex_index<<std::endl;

	// std::cout<<"Press: ";
	// std::cin.get();

	int deg = out_degree(v,graph);

	int complete_drop = rand()%5;

	if(isFringeNode(graph[v].state,epsilon) && complete_drop==0)
	{
		clear_vertex(v, graph);
		remove_vertex(v, graph);
	}
	else 
	{
		if(deg == 2)
		{
			NeighborIter ai, ai_end;
			boost::tie(ai, ai_end) = adjacent_vertices(v, graph); 
			
			Vertex a = *ai;
			++ai;
			Vertex b = *ai;
			++ai;

			if(isEqual(graph[a].state[0],graph[b].state[0])  
				|| isEqual(graph[a].state[1],graph[b].state[1])	)
				std::pair<Edge,bool> curEdge = boost::add_edge(a,b, graph);

			clear_vertex(v, graph);
			remove_vertex(v, graph);
		}
		if(deg == 3)
		{
			NeighborIter ai, ai_end;
			boost::tie(ai, ai_end) = adjacent_vertices(v, graph); 
			
			Vertex a = *ai;
			++ai;
			Vertex b = *ai;
			++ai;
			Vertex c = *ai;
			++ai;

			if(isEqual(graph[a].state[0],graph[b].state[0]))
				std::pair<Edge,bool> curEdge = boost::add_edge(a,b, graph);	
			else if(isEqual(graph[a].state[0],graph[c].state[0]))
				std::pair<Edge,bool> curEdge = boost::add_edge(a,c, graph);	
			else if(isEqual(graph[a].state[1],graph[b].state[1]))
				std::pair<Edge,bool> curEdge = boost::add_edge(a,b, graph);	
			else if(isEqual(graph[a].state[1],graph[c].state[1]))
				std::pair<Edge,bool> curEdge = boost::add_edge(a,c, graph);	
			else
				std::pair<Edge,bool> curEdge = boost::add_edge(b,c, graph);	

			clear_vertex(v, graph);
			remove_vertex(v, graph);
		}
		if(deg == 4)
		{
			NeighborIter ai, ai_end;
			boost::tie(ai, ai_end) = adjacent_vertices(v, graph); 
			
			Vertex a = *ai;
			++ai;
			Vertex b = *ai;
			++ai;
			Vertex c = *ai;
			++ai;
			Vertex d = *ai;
			++ai;

			if(isEqual(graph[a].state[0],graph[b].state[0]))
				std::pair<Edge,bool> curEdge = boost::add_edge(a,b, graph);	
			else if(isEqual(graph[a].state[0],graph[c].state[0]))
				std::pair<Edge,bool> curEdge = boost::add_edge(a,c, graph);	
			else if(isEqual(graph[a].state[0],graph[d].state[0]))
				std::pair<Edge,bool> curEdge = boost::add_edge(a,d, graph);	
			else if(isEqual(graph[a].state[1],graph[b].state[1]))
				std::pair<Edge,bool> curEdge = boost::add_edge(a,b, graph);	
			else if(isEqual(graph[a].state[1],graph[c].state[1]))
				std::pair<Edge,bool> curEdge = boost::add_edge(a,c, graph);	
			else
				std::pair<Edge,bool> curEdge = boost::add_edge(a,d, graph);	

			clear_vertex(v, graph);
			remove_vertex(v, graph);
		}
	}
}


int main(int argc, char* argv[])
{
	srand(time(NULL));
	po::options_description desc("Select Epsilon, Drop Value & Output Filename");
	desc.add_options()
		("help", "produce help message")
		("epsilon,e",po::value<double>()->required(), "epsilon value")
		("drop_value,d",po::value<int>()->required(), "drop value")
		("output_graph,g",po::value<std::string>()->required(), " path to graphml file")
	;

	po::variables_map vm;
	po::store(po::parse_command_line(argc,argv,desc), vm);

	if(vm.count("help")) {
		std::cout<<desc<<std::endl;
		return 1;
	}

	std::string output_filepath=vm["output_graph"].as<std::string>();
	double epsilon = vm["epsilon"].as<double>();
	int drop_value = vm["drop_value"].as<int>();
	
	for(int number=0; number<10; number++)
	{	
		// Space Information
		Graph graph;

		std::unordered_map<Eigen::VectorXd, Vertex, matrix_hash<Eigen::VectorXd>> configToNodeMap;
		
		for(double x=epsilon; x<0.999999999;x+=epsilon)
			for(double y=epsilon; y<0.999999999;y+=epsilon)
			{
				Eigen::VectorXd config(2);
				config << x,y;
				Vertex new_node;
				new_node = add_vertex(graph);
				graph[new_node].state = config;
				graph[new_node].vertex_index = boost::num_vertices(graph) - 1;
				configToNodeMap[config]=new_node;
			}

		for(double x=epsilon; x<0.999999999;x+=epsilon)
			for(double y=epsilon; y<0.999999999;y+=epsilon)
			{
				Eigen::VectorXd config(2);
				config << x, y;
				
				if(x+epsilon < 0.999999999)
				{
					Eigen::VectorXd right_config(2);
					right_config << x + epsilon, y;

					std::pair<Edge,bool> curEdge = 
						boost::add_edge(configToNodeMap[config], configToNodeMap[right_config], graph);
				}

				if(y+epsilon < 0.999999999)
				{
					Eigen::VectorXd down_config(2);
					down_config << x, y + epsilon;

					std::pair<Edge,bool> curEdge = 
						boost::add_edge(configToNodeMap[config], configToNodeMap[down_config], graph);
				}	
			}

		for(int i=0;i<drop_value;i++)
		{
			// display_graph(graph);
			drop_vertices(graph,epsilon);
			// displayGraphOnImage(graph, image);
		}

		while(1)
		{
			VertexIter vi, vi_end;
			boost::tie(vi,vi_end)=vertices(graph);
			int flag=0;
			while(vi!=vi_end)
			{
				Vertex v = *vi;
				int deg = out_degree(v,graph);
				if(deg==0)
				{
					remove_vertex(v, graph);
					flag=1;
					break;				
				}
				if(deg==2)
				{
					NeighborIter ai, ai_end;
					boost::tie(ai, ai_end) = adjacent_vertices(v, graph); 
					
					Vertex a = *ai;
					++ai;
					Vertex b = *ai;
					++ai;

					if(isEqual(graph[a].state[0],graph[b].state[0])  
						|| isEqual(graph[a].state[1],graph[b].state[1])	)
					{
						std::pair<Edge,bool> curEdge = boost::add_edge(a,b, graph);
						clear_vertex(v, graph);
						remove_vertex(v, graph);
						flag=1;
						break;
					}
				}
				++vi;
			}
			if(flag==0)
				break;
		}

		// std::cout<<"Press [ENTER] to display graph:";
		// std::cin.get();

		std::cout<<"#Vertices: "<<boost::num_vertices(graph)<<
			"#Edges: "<<boost::num_edges(graph)<<std::endl;

		// display_graph(graph);

		Graph new_graph;	
		
		std::unordered_map<Vertex,Vertex> indToIndMap;

		VertexIter vi, vend;
		for (boost::tie(vi, vend) = vertices(graph); vi != vend; ++vi) 
		{
  			Vertex new_node;
			new_node = add_vertex(new_graph);
			new_graph[new_node].state = graph[*vi].state;
			new_graph[new_node].vertex_index = boost::num_vertices(new_graph) - 1;
			indToIndMap[*vi] = new_node;
  		}

	  	EdgeIter ei, ei_end;
		for(boost::tie(ei,ei_end)=edges(graph);ei!=ei_end;++ei) 
		{
			std::pair<Edge,bool> curEdge = boost::add_edge(indToIndMap[source(*ei,graph)],
				indToIndMap[target(*ei,graph)], new_graph);
		}	

		write_graph(new_graph,output_filepath+"graph"+std::to_string(number)+".graphml",2);
	}
	
	return 0;
}