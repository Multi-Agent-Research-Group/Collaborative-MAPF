#ifndef TPQ_
#define TPQ_

#include<bits/stdc++.h>
#include <boost/functional/hash.hpp>

#define make_search_state(a,b,c,d) std::make_pair(std::make_pair(a,b),std::make_pair(c,d))
#define search_state std::pair< std::pair<Vertex,int>, std::pair<int,bool> >
#define search_state_vertex(t) t.first.first
#define search_state_timestep(t) t.first.second
#define search_state_tasks_completed(t) t.second.first
#define search_state_in_delivery(t) t.second.second

using namespace std;

namespace CMAPF {

using namespace BGL_DEFINITIONS;

struct SearchState
{
	Vertex vertex;
	int timestep;
	int tasks_completed;
	bool in_delivery;

	SearchState()
		: vertex(0), timestep(0), tasks_completed(0), in_delivery(false) {}

	SearchState(Vertex _vertex, int _timestep, int _tasks_completed, bool _in_delivery) 
		: vertex(_vertex), timestep(_timestep), tasks_completed(_tasks_completed), in_delivery(_in_delivery) {}

	bool operator==(const SearchState &other) const
	{ 
		return (vertex == other.vertex && timestep == other.timestep
			&& tasks_completed == other.tasks_completed && in_delivery == other.in_delivery );
	}

};

struct state_hash
{
  std::size_t operator()(const SearchState& k) const
  {
      using boost::hash_value;
      using boost::hash_combine;

      // Start with a hash value of 0    .
      std::size_t seed = 0;

      // Modify 'seed' by XORing and bit-shifting in
      // one member of 'SearchState' after the other:
      hash_combine(seed,hash_value(k.vertex));
      hash_combine(seed,hash_value(k.timestep));
      hash_combine(seed,hash_value(k.tasks_completed));
      hash_combine(seed,hash_value(k.in_delivery));

      // Return the result.
      return seed;
  }
};


class timePriorityQueue
{
private:


	struct element
	{
		double key1;
		double key2;
		SearchState s;
		element(double _key1, double _key2, SearchState _s): key1(_key1), key2(_key2), s(_s) {} 
		inline bool operator < (const element &b) const 
		{
        	if(key1<b.key1)
				return true;
			else if(key1 == b.key1 && key2<b.key2)
				return true;
			else
				return false;
		}
	};
	vector <element> PQ;

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
	timePriorityQueue()
	{ 
		SearchState s = SearchState();
		element a(-1,-1,s);
		PQ.push_back(a);
	}
	void reset()
	{
		PQ.clear();
		SearchState s = SearchState();
		element a(-1,-1,s);
		PQ.push_back(a);
	}
	int PQsize()
	{
		return PQ.size()-1;
	}
	pair<double,double> topKey()
	{
		return make_pair(PQ[1].key1,PQ[1].key2);
	}
	SearchState pop()
	{
		SearchState s=PQ[1].s;
		PQ[1]=PQ[PQ.size()-1];
		PQ.erase(PQ.end()-1);
		min_heapify(1);
		return s;
	}
	void insert(SearchState s, double k1, double k2)
	{
		// std::cout<<"Inserting : "<<v<<std::endl;
		element a(k1,k2,s);
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
	void remove(SearchState s)
	{
		int i;
		for(i=1;i<PQ.size();i++)
			if(PQ[i].s.vertex==s.vertex && PQ[i].s.timestep == s.timestep 
				&& PQ[i].s.tasks_completed == s.tasks_completed && PQ[i].s.in_delivery == s.in_delivery)
				break;
		swap(PQ[i],PQ[PQ.size()-1]);
		PQ.erase(PQ.end()-1);
		// printPQ();
		min_heapify(i);
		// printPQ();
	}
	bool contains(SearchState s)
	{
		for(int i=1;i<PQ.size();i++)
			if(PQ[i].s.vertex==s.vertex && PQ[i].s.timestep == s.timestep 
				&& PQ[i].s.tasks_completed == s.tasks_completed && PQ[i].s.in_delivery == s.in_delivery)
				break;
		return false;
	}
	void printPQ()
	{
		cout<<"Elements: "<<endl;
		for(int i=1;i<PQ.size();i++)
			cout<<"( Vertex: "<<PQ[i].s.vertex<<", Timestep: "<<PQ[i].s.timestep
				<<", Tasks Completed: "<<PQ[i].s.tasks_completed<<", In Delivery: "<<PQ[i].s.in_delivery
				<<", Key1: "<<PQ[i].key1<<", Key2: "<<PQ[i].key2<<"), ";
		cout<<endl;
	}
};

} // namespace CMAPF

#endif 