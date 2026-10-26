from .geometry import euclidean_distance


#all Weights = 1 as each round = 1 hop

#We only need to find D: depth of topology
def bfs(N_u, graph):

    D_max = 0
    disconnected = False #to detect disconnected graph

    #we want to check every agent longest path and pick the longest
    for i in range(N_u):
        queue = [i] #neighbor nodes of processed node
        level = [-1] * N_u #has two jobs: mark a node is visited and its depth

        level[i] = 0 #root node has depth 0
        #we only care about depth, thus stop as soon as every node is visited
        while queue:

            curr = queue.pop(0)

            for j in range(N_u):
                if level[j] == -1 and graph[curr][j] == 1:
                    level[j] = level[curr] + 1
                    queue.append(j)
            
        if -1 in level:
            disconnected = True


        D_max = max(D_max, max(level))

    return (disconnected, D_max)


def build_graph(agent_dict, comm_range):
    N_u = len(agent_dict)
    graph = [[0] * N_u for _ in range(N_u)]

    for i in range(N_u):
        agent_i = agent_dict[i]
        for j in range(i):
            agent_j = agent_dict[j]

            distance = euclidean_distance(agent_i.position, agent_j.position)
            #undirected graph, thus symmetric
            if distance <= comm_range:
                graph[i][j] = 1
                graph[j][i] = 1

    return graph
#hint: comm_range is the range of communication per hop