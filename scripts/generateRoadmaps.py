# Import standard python libraries
import networkx as nx
import numpy
import math
# import helper
import time

# Halton Sequence Generator
def halton_sequence_value(index, base):
    
    result = 0
    f = 1

    while index > 0:
        f = f*1.0/base
        result = result + f*(index % base)
        index = index/base
    
    return result

def calc_weight_states(s1, s2):
    # print(s1,s2)
    # print("Distance: ",math.sqrt(numpy.sum(numpy.power(s1-s2,2))))
    return math.sqrt(numpy.sum(numpy.power(s1-s2,2)))

# Wrap the values around 0 and 1
def wrap_around(coordinate):

    for i in range(numpy.size(coordinate)):
        while coordinate[i] > 1.0:
            coordinate[i] = coordinate[i] - 1.0
        while coordinate[i] < 0:
            coordinate[i] = 1.0 + coordinate[i]

    return coordinate

# def update_value(val_list):
#     for i in range(0,len(val_list)):
#         val_list[i] = 2*math.pi*val_list[i]
#     return val_list
#     # return

# Halton Graph Generator
def euclidean_halton_graph(n, radius, bases, lower, upper, offset, space_dim, enable=True):

    G = nx.Graph()
    t1 = time.time()

    position = {i-1 : wrap_around(numpy.array([halton_sequence_value(i,base) for base in bases]) + offset) for i in range(1, n+1)}

#     for i in range(len(position)):
#         position[i] = update_value(position[i])

    if space_dim == 2:
        state = {i: `position[i][0]` + ' ' + `position[i][1]` for i in position.keys()}
    
    if space_dim == 3:
        state = {i: `position[i][0]` + ' ' + `position[i][1]`+ ' ' +`position[i][2]` for i in position.keys()}

    if space_dim == 4:
        state = {i: `position[i][0]` + ' ' + `position[i][1]` + ' ' + `position[i][2]` + ' ' + `position[i][3]` for i in position.keys()}
    if space_dim == 5:
        state = {i: `position[i][0]` + ' ' + `position[i][1]` + ' ' + `position[i][2]` + ' ' + `position[i][3]` + ' ' + `position[i][4]` for i in position.keys()}
    if space_dim == 7:
        state = {i: str(position[i][0]) + ' ' + str(position[i][1]) + ' ' + str(position[i][2]) + ' ' + str(position[i][3]) + ' ' + str(position[i][4]) + ' ' + str(position[i][5]) + ' ' + str(position[i][6]) for i in position.keys()}


    t2 = time.time()
    print("time here = ", t2-t1)
    for i in range(n):
        node_id = i
        G.add_node(node_id, state = state[i])

    for i in range(n-1):     
        for j in range(i+1,n):
            w = calc_weight_states(position[i],position[j])
            if w < radius:
                G.add_edge(i, j) 
            # else:
            #     print("w = ",w)
    return G

# Main Function
if __name__ == "__main__":

    space_dim = 2

    if space_dim == 2:
        bases = [2,3]
    if space_dim == 3:
        bases = [2,3,5]
    if space_dim == 4:
        bases = [2,3,5,7]
    if space_dim == 5:
        bases = [2,3,5,7,11]
    if space_dim == 7:
        bases = [2,3,5,7,11,13,17]

    lower = numpy.ones(space_dim)*0
    upper = numpy.ones(space_dim)

    # Settings
    h_points = [10]

    i = 0

    
    for halton_points in h_points:
        print("h_points = ", halton_points)
        disc_radius = 0.5 # 3000
        # disc_radius = math.pi*1.1 # 200
        print i
        numpy.random.seed()
        offset = numpy.random.random_sample(space_dim,)
        riskmapFile = '../data/graphs/lol' + `space_dim` + 'D' + `halton_points` + '_' + `i+1` + '.graphml'
        # Example: halton2D2000_0.graphml

        # Generate the graph
        t1 = time.time()
        G = euclidean_halton_graph(halton_points, disc_radius, bases, lower, upper, offset, space_dim)
        t2 = time.time()
        print("time taken = ", t2-t1)
        nx.write_graphml(G, riskmapFile)