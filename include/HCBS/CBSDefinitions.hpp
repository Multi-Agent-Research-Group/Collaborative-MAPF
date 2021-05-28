#ifndef CBS_DEFINITIONS_
#define CBS_DEFINITIONS_
#include <bits/stdc++.h>
#include "BGLDefinitions.hpp"
#include "LoadGraphfromFile.hpp"
#include <boost/functional/hash.hpp>

namespace HCBS {

using namespace BGL_DEFINITIONS;

using namespace std;


struct Constraint
{
	int constraint_type;  // 1 -> vertex constraint 2 -> edge constraint
	Vertex v;
	Edge e;	//edge
	size_t t; //time . for edge, time is time of target vertex

	Constraint() : constraint_type(1) {}
	Constraint(Vertex _v, size_t _t) : v(_v), t(_t), constraint_type(1) {}
	Constraint(Edge _e, size_t _t) : e(_e), t(_t), constraint_type(2) {}

	bool operator==(const Constraint &other) const
	{ 
		if (constraint_type != other.constraint_type) return false;
		if (t!=other.t) return false;
		if (constraint_type == 1) return v == other.v;
		else if (constraint_type == 2) return e == other.e;
	}
};

struct PathQuery
{
  Vertex start;
  Vertex goal;
  
  std::vector<Constraint> constraints;

  int initial_timestep;
  int final_timestep;

  bool operator==(const PathQuery &other) const
  { 
  	// sort(constraints.begin(), constraints.end());
  	// sort(other.constraints.begin(), other.constraints.end());
  	return (start == other.start
            && goal == other.goal
            && constraints == other.constraints
            && initial_timestep == other.initial_timestep
            && final_timestep == other.final_timestep);
  }

  // PathQuery (Vertex _start, Vertex _goal, int _initial_timestep, int _final_timestep): start(_start), goal(_goal), initial_timestep(_initial_timestep), final_timestep(_final_timestep) {}
  PathQuery (Vertex _start, Vertex _goal, int _initial_timestep, int _final_timestep, std::vector <Constraint> _constraints): 
  start(_start), goal(_goal), initial_timestep(_initial_timestep), final_timestep(_final_timestep), constraints(_constraints) {}
};

struct PathQueryHasher
{
  std::size_t operator()(const PathQuery& k) const
  {
      using boost::hash_value;
      using boost::hash_combine;

      // Start with a hash value of 0    .
      std::size_t seed = 0;

      // Modify 'seed' by XORing and bit-shifting in
      // one member of 'Key' after the other:
      hash_combine(seed,hash_value(k.start));
      hash_combine(seed,hash_value(k.goal));
      for(int i=0; i<k.constraints.size(); i++){
      	hash_combine(seed,hash_value(k.constraints[i].t));
      	hash_combine(seed,hash_value(k.constraints[i].constraint_type));
      	if(k.constraints[i].constraint_type == 1){
      		hash_combine(seed,hash_value(k.constraints[i].v));
      	}
      	else if(k.constraints[i].constraint_type == 2){
      		hash_combine(seed,hash_value(k.constraints[i].e.m_source));
      		hash_combine(seed,hash_value(k.constraints[i].e.m_target));
      	}
      }
      // 
      hash_combine(seed,hash_value(k.initial_timestep));
      hash_combine(seed,hash_value(k.final_timestep));
      // Return the result.
      return seed;
  }
};

struct pair_hash {
    template <class T1, class T2>
    std::size_t operator () (const std::pair<T1,T2> &p) const {
        auto h1 = std::hash<T1>{}(p.first);
        auto h2 = std::hash<T2>{}(p.second);

        // Mainly for demonstration purposes, i.e. works but is overly simple
        // In the real world, use sth. like boost.hash_combine
        return h1 ^ h2;  
    }
}; 


struct Element
{
	double cost;
	std::vector<std::vector< Constraint >> constraints;
	std::vector<std::vector<Vertex>> shortestPaths;

	Element(double _cost, std::vector<std::vector<Constraint>> _constraints, std::vector<std::vector<Vertex>> _shortestPaths): 
		cost(_cost), constraints(_constraints), shortestPaths(_shortestPaths) 
		{}

	inline bool operator < (const Element &b) const 
	{
    	if(cost<b.cost)
			return true;
		else
			return false;
	}
};

class CBSPriorityQueue
{
private:
	vector <Element> PQ;
	size_t mNumAgents;

	void min_heapify(int x)
	{
		int l=2*x;
		int r=2*x+1;
		int smallest = x;
		if(l<=(PQ.size()-1) && PQ[l]<PQ[x])
			smallest = l;
		if(r<=(PQ.size()-1) && PQ[r]<PQ[smallest])
			smallest = r;
		if(smallest!=x)
		{
			swap(PQ[smallest],PQ[x]);
			min_heapify(smallest);
		}
	}

public:
	CBSPriorityQueue(size_t numAgents)
	{ 
		mNumAgents = numAgents;
		Element a(-1.0,std::vector<std::vector<Constraint>>(mNumAgents, std::vector<Constraint>()),std::vector<std::vector<Vertex>>(mNumAgents,std::vector<Vertex>()));
		PQ.push_back(a);
	}
	void reset()
	{
		PQ.clear();
		Element a(-1.0,std::vector<std::vector<Constraint>>(mNumAgents, std::vector<Constraint>()),std::vector<std::vector<Vertex>>(mNumAgents,std::vector<Vertex>()));
		PQ.push_back(a);
	}
	int PQsize()
	{
		return PQ.size()-1;
	}
	double topKey()
	{
		return PQ[1].cost;
	}
	Element pop()
	{
		Element temp=PQ[1];
		PQ[1]=PQ[PQ.size()-1];
		PQ.erase(PQ.end()-1);
		min_heapify(1);
		return temp;
	}
	void insert(double _cost, std::vector<std::vector<Constraint>> _constraints, std::vector<std::vector<Vertex>> _shortestPaths)
	{
		Element a(_cost, _constraints, _shortestPaths);
		PQ.push_back(a);
		// printPQ();
		int i=PQ.size()-1;
		while((i/2)>0)
		{
			if(PQ[i/2]<PQ[i])
				break;
			else
			{
				swap(PQ[i/2],PQ[i]);
				i=i/2;
			}
		}

	}
	void print()
	{
		cerr<<"\n\n --------------------------------------- CBS PQ Elements: "<<endl;
		for(int i=1;i<PQ.size();i++)
		{
			cerr<<PQ[i].cost<<" ";

			// cout<<"Constraints: "<<endl;
			// //print constraints
			// for(int agent_id=0; agent_id<mNumAgents; agent_id++)
			// {
			// 	std::cout<<" Agent: "<<agent_id<<" - ";
			// 	for(auto it=PQ[i].constraints[agent_id].begin(); it != PQ[i].constraints[agent_id].end(); it++)
			// 	{
			// 		if((*it).constraint_type == 1)
			// 			cout<<"Vertex: "<<(*it).v<<" Time: "<<(*it).t<<endl;
			// 		else
			// 			cout<<"Edge: "<<(*it).e<<" Time: "<<(*it).t<<endl;
			// 	}
			// }

			// // cout<<"shortestPaths: "<<endl;
			// // not print paths as indexmap needed
			// cout<<endl;
		}
		cerr<<endl;
	}
};

} // namespace HCBS

#endif