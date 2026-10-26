class Agent:
    def __init__(self, agent_id, position, num_tasks, num_agents, speed=1.0):
        self.id = agent_id
        self.position = position
        self.speed = speed  # distance units per time unit; travel time = distance / speed
        self.bundle = []  # task id goes here
        self.path = []  # task id goes here
        self.winning_agent_list = [-1] * num_tasks
        self.winning_bid_list = [0.0] * num_tasks
        #paper assume no delay
        self.timestamp_list = [-1] * num_agents #store the reception time (most up to date)

    def set_winner(self, j, agent_id, bid):
        self.winning_agent_list[j] = agent_id
        self.winning_bid_list[j] = bid

    def set_path(self, index, task_id):
        self.path.insert(index, task_id)
    
    def set_bundle(self, task_id):
        self.bundle.append(task_id)
        

class Task:
    def __init__(self, task_id, position, static_score, discount_factor):
        self.id = task_id
        self.position = position
        self.static_score = static_score
        self.discount_factor = discount_factor

class Message:
    #the send time is timestamp_list[sender_id], so it needs no field of its own
    def __init__(self, sender_id, winning_bid_list, winning_agent_list, timestamp_list):
        self.sender_id = sender_id
        self.winning_bid_list = winning_bid_list 
        self.winning_agent_list = winning_agent_list
        self.timestamp_list = timestamp_list

class Records:
    def __init__(self, converged, disconnected, T_c, N_min, D, comm_range):
        self.converged =  converged
        self.disconnected = disconnected
        self.T_c = T_c
        self.N_min = N_min
        self.D = D 
        self.comm_range = comm_range

    def get_ratio(self):
        if self.disconnected:
            return None

        return self.T_c/(self.N_min * self.D)

