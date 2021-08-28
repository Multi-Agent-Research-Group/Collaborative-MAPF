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

namespace PCCBS {

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

bool compareCollaborationConstraints(SearchState &i1, SearchState &i2)
{
    return (i1.timestep < i2.timestep);
}

struct state_hash
{
  std::size_t operator()(const SearchState& k) const
  {
 //  	size_t h(14695981039346656037UL); // Offset basis
	// unsigned i(0);
	// Start with a hash value of 0    .
	// std::size_t seed = 0;
	// std::vector<size_t> hash_elements;
	return size_t(k.in_delivery)+2*size_t(k.tasks_completed)+size_t(k.vertex)*2*16+size_t(k.timestep)*2*16*1024;
	// hash_elements.push_back();
	// hash_elements.push_back(k.timestep);
	// hash_elements.push_back(k.tasks_completed);
	// hash_elements.push_back(k.in_delivery);
	// // Modify 'seed' by XORing and bit-shifting in
	// // one member of 'SearchState' after the other:
	// for(int hi=0; hi<hash_elements.size(); hi++)
	// {
	// 	size_t h1(hash_elements[hi]);
	// 	uint8_t c[sizeof(size_t)];
	// 	memcpy(c,&h1,sizeof(size_t));
	// 	for(unsigned j(0); j<sizeof(size_t); ++j){
	// 		//hash[k*sizeof(uint64_t)+j]=((int)c[j])?c[j]:1; // Replace null-terminators in the middle of the string
	// 		h=h^c[j]; // Xor with octet
	// 		h=h*1099511628211; // multiply by the FNV prime
	// 	}
	// }
	// Return the result.
	// return h;
  }
};

size_t get_hash_state(SearchState k){
	return size_t(k.in_delivery)+2*size_t(k.tasks_completed)+size_t(k.vertex)*2*16+size_t(k.timestep)*2*16*1024;
}

struct agent_state_pair_hash
{
  std::size_t operator()(const std::pair<int, std::pair <SearchState,SearchState>>& k1) const
  {
      using boost::hash_value;
      using boost::hash_combine;

      // Start with a hash value of 0    .
      std::size_t seed = 0;

      // Modify 'seed' by XORing and bit-shifting in
      // one member of 'SearchState' after the other:
      hash_combine(seed, hash_value(get_hash_state(k1.second.first)));
      hash_combine(seed, hash_value(get_hash_state(k1.second.second)));
      hash_combine(seed, hash_value((k1.first)));
      return seed;
  }
};

struct time_state_hash
{
  std::size_t operator()(const std::pair<int, SearchState>& k1) const
  {
      // using boost::hash_value;
      // using boost::hash_combine;

      // Start with a hash value of 0    .
      // std::size_t seed = 0;

      // Modify 'seed' by XORing and bit-shifting in
      // one member of 'SearchState' after the other:
      // hash_combine(seed, hash_value(k1.first));
      // SearchState k = k1.second;
      // hash_combine(seed,hash_value(k.vertex));
      // hash_combine(seed,hash_value(k.timestep));
      // hash_combine(seed,hash_value(k.tasks_completed));
      // hash_combine(seed,hash_value(k.in_delivery));
      // std::vector<size_t> hash_elements;
	  return size_t(k1.second.in_delivery)+2*size_t(k1.second.tasks_completed)+size_t(k1.second.vertex)*2*16+
	  size_t(k1.second.timestep)*2*16*1024+size_t(k1.first)*2*16*1024*1024;
      // Return the result.
      // return seed;
  }
};
// struct state_hash
// {
//   std::size_t operator()(const SearchState& k) const
//   {
//       using boost::hash_value;
//       using boost::hash_combine;

//       // Start with a hash value of 0    .
//       std::size_t seed = 0;

//       // Modify 'seed' by XORing and bit-shifting in
//       // one member of 'SearchState' after the other:
//       hash_combine(seed,hash_value(k.vertex));
//       hash_combine(seed,hash_value(k.timestep));
//       hash_combine(seed,hash_value(k.tasks_completed));
//       hash_combine(seed,hash_value(k.in_delivery));

//       // Return the result.
//       return seed;
//   }
// };

class timePriorityQueue
{
private:


	struct element
	{
		std::vector<double> keys;
		SearchState s;
		element(SearchState _s, std::vector<double> _keys): s(_s), keys(_keys) {} 
		inline bool operator < (const element &b) const 
		{
			if(keys.size() != b.keys.size())
			{
				std::cerr<<"ERROR: Key sizes are not equal!";
				std::cin.get();
			}
			for(int i=0; i<keys.size(); i++){
			// for(int i=0; i<1; i++){
				if((keys[i] - b.keys[i]) < -0.01)
					return true;
				else if((keys[i] - b.keys[i]) > 0.01)
					return false;
			}
			// return true;
			return s.vertex < b.s.vertex;

			// if(key1<b.key1)
			// 	return true;
			// else if(std::abs(key1-b.key1)<EPS && key2<b.key2)
			// 	return true;
			// return false;
			// if(std::abs(key1-b.key1)<0.001)
			// {
			// 	// std::cout<<s.vertex<<" "<<b.s.vertex<<std::endl;
			// 	return s.vertex < b.s.vertex;
			// }
			// return key1 < b.key1;
   //      	if(key1<b.key1)
			// 	return true;
			// else if(key1 == b.key1 && key2<b.key2)
			// 	return true;
			// else
			// 	return false;
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
		element a(s,std::vector<double>());
		PQ.push_back(a);
	}
	void reset()
	{
		PQ.clear();
		SearchState s = SearchState();
		element a(s,std::vector<double>());
		PQ.push_back(a);
	}
	int PQsize()
	{
		return PQ.size()-1;
	}
	std::vector<double> topKey()
	{
		return PQ[1].keys;
	}
	SearchState pop()
	{
		SearchState s=PQ[1].s;
		PQ[1]=PQ[PQ.size()-1];
		PQ.erase(PQ.end()-1);
		min_heapify(1);
		// std::cout << PQ[1].keys[0] << std::endl;
		return s;
	}
	void insert(SearchState s, std::vector<double> k)
	{
		// std::cout<<"Inserting : "<<v<<std::endl;
		element a(s,k);
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
	void remove(SearchState &s)
	{
		int i;
		for(i=1;i<PQ.size();i++)
			if(PQ[i].s == s)
				break;
		swap(PQ[i],PQ[PQ.size()-1]);
		PQ.erase(PQ.end()-1);
		// printPQ();
		min_heapify(i);
		// printPQ();
	}
	bool contains(SearchState &s)
	{
		for(int i=1;i<PQ.size();i++)
			if(PQ[i].s == s)
				break;
		return false;
	}
	void printPQ()
	{
		cout<<"Elements: "<<endl;
		for(int i=1;i<PQ.size();i++)
		{
			cout<<"( Vertex: "<<PQ[i].s.vertex<<", Timestep: "<<PQ[i].s.timestep
				<<", Tasks Completed: "<<PQ[i].s.tasks_completed<<", In Delivery: "<<PQ[i].s.in_delivery
				<<", Keys: ";

			for(int j=0; j<PQ[i].keys.size(); j++)
				cout<<PQ[i].keys[j]<<" ";
			cout<<"\n";
		}
	}
};
// class timePriorityQueue
// {
// private:


// 	struct element
// 	{
// 		double key1;
// 		double key2;
// 		SearchState s;
// 		element(double _key1, double _key2, SearchState _s): key1(_key1), key2(_key2), s(_s) {} 
// 		inline bool operator < (const element &b) const 
// 		{
// 			if(std::abs(key1-b.key1)<0.001)
// 			{
// 				// std::cout<<s.vertex<<" "<<b.s.vertex<<std::endl;
// 				return s.vertex < b.s.vertex;
// 			}
// 			return key1 < b.key1;
//    //      	if(key1<b.key1)
// 			// 	return true;
// 			// else if(key1 == b.key1 && key2<b.key2)
// 			// 	return true;
// 			// else
// 			// 	return false;
// 		}
// 	};
// 	vector <element> PQ;

// 	void min_heapify(int x)
// 	{
// 		int l=2*x;
// 		int r=2*x+1;
// 		int smallest = x;
// 		if(l<=(PQ.size()-1) && PQ[l]<PQ[x])
// 			smallest = l;
// 		if(r<=(PQ.size()-1) && PQ[r]<PQ[smallest])
// 			smallest = r;
// 		if(smallest!=x)
// 		{
// 			swap(PQ[smallest],PQ[x]);
// 			min_heapify(smallest);
// 		}
// 	}

// public:
// 	timePriorityQueue()
// 	{ 
// 		SearchState s = SearchState();
// 		element a(-1,-1,s);
// 		PQ.push_back(a);
// 	}
// 	void reset()
// 	{
// 		PQ.clear();
// 		SearchState s = SearchState();
// 		element a(-1,-1,s);
// 		PQ.push_back(a);
// 	}
// 	int PQsize()
// 	{
// 		return PQ.size()-1;
// 	}
// 	pair<double,double> topKey()
// 	{
// 		return make_pair(PQ[1].key1,PQ[1].key2);
// 	}
// 	SearchState pop()
// 	{
// 		SearchState s=PQ[1].s;
// 		PQ[1]=PQ[PQ.size()-1];
// 		PQ.erase(PQ.end()-1);
// 		min_heapify(1);
// 		return s;
// 	}
// 	void insert(SearchState s, double k1, double k2)
// 	{
// 		// std::cout<<"Inserting : "<<v<<std::endl;
// 		element a(k1,k2,s);
// 		PQ.push_back(a);
// 		// printPQ();
// 		int i=PQ.size()-1;
// 		while((i/2)>0)
// 		{
// 			if(PQ[i/2]<PQ[i])
// 				break;
// 			else
// 			{
// 				swap(PQ[i/2],PQ[i]);
// 				i=i/2;
// 			}
// 		}

// 	}
// 	void remove(SearchState s)
// 	{
// 		int i;
// 		for(i=1;i<PQ.size();i++)
// 			if(PQ[i].s.vertex==s.vertex && PQ[i].s.timestep == s.timestep 
// 				&& PQ[i].s.tasks_completed == s.tasks_completed && PQ[i].s.in_delivery == s.in_delivery)
// 				break;
// 		swap(PQ[i],PQ[PQ.size()-1]);
// 		PQ.erase(PQ.end()-1);
// 		// printPQ();
// 		min_heapify(i);
// 		// printPQ();
// 	}
// 	bool contains(SearchState s)
// 	{
// 		for(int i=1;i<PQ.size();i++)
// 			if(PQ[i].s.vertex==s.vertex && PQ[i].s.timestep == s.timestep 
// 				&& PQ[i].s.tasks_completed == s.tasks_completed && PQ[i].s.in_delivery == s.in_delivery)
// 				break;
// 		return false;
// 	}
// 	void printPQ()
// 	{
// 		cout<<"Elements: "<<endl;
// 		for(int i=1;i<PQ.size();i++)
// 			cout<<"( Vertex: "<<PQ[i].s.vertex<<", Timestep: "<<PQ[i].s.timestep
// 				<<", Tasks Completed: "<<PQ[i].s.tasks_completed<<", In Delivery: "<<PQ[i].s.in_delivery
// 				<<", Key1: "<<PQ[i].key1<<", Key2: "<<PQ[i].key2<<"), ";
// 		cout<<endl;
// 	}
// };

} // namespace PCCBS

#endif 