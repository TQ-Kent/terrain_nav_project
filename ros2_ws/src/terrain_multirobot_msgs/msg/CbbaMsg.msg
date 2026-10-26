# One CBBA broadcast (Choi, Brunet, How 2009): what a robot believes about every task.
int32 sender_id
# Highest known bid per task (index = task id)
float64[] winning_bid
# Robot that holds that bid, -1 = nobody
int32[] winning_agent
# Latest message stamp heard from each robot (index = robot id); own entry = sender's round
int32[] timestamps
