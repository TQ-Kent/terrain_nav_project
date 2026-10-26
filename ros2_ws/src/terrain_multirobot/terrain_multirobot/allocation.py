"""One robot's side of CBBA without any ROS code, so it can be tested on its own.

The reference `simulate()` runs every agent in lock-step. A real robot has no shared round counter,
so each robot runs its own loop: read the messages that arrived, resolve conflicts, bid, broadcast.
"""
from .cbba.bundle import build_bundle
from .cbba.consensus import consume_message, produce_message, release
from .cbba.models import Agent, Message, Task


class RobotAllocator:
    """Holds the CBBA state of one robot. Call `step()` at a fixed rate."""

    def __init__(self, robot_id, position, tasks, num_robots, speed=1.0, max_bundle=2, quiet_steps=6):
        """tasks: [(x, y, value, discount)] indexed by task id."""
        self.agent = Agent(robot_id, position, len(tasks), num_robots, speed=speed)
        self.tasks = {j: Task(j, (x, y), value, discount) for j, (x, y, value, discount) in enumerate(tasks)}
        self.max_bundle = max_bundle
        self.quiet_steps = quiet_steps
        self.round = 0
        self.quiet = 0
        self._last = None

    @property
    def id(self):
        return self.agent.id

    def _state(self):
        a = self.agent
        return (tuple(a.bundle), tuple(a.path), tuple(a.winning_agent_list), tuple(a.winning_bid_list))

    def step(self, inbox):
        """inbox: Messages received since the last step. Returns the Message to broadcast."""
        self.round += 1
        for msg in inbox:
            if msg.sender_id != self.id:
                consume_message(self.agent, msg)
        release(self.agent)
        build_bundle(self.agent, self.tasks, self.max_bundle)
        msg = produce_message(self.agent, self.round)
        state = self._state()
        self.quiet = self.quiet + 1 if state == self._last else 0
        self._last = state
        return msg

    def heard_everyone(self):
        return all(t >= 0 for i, t in enumerate(self.agent.timestamp_list) if i != self.id)

    def converged(self):
        """Nothing changed for `quiet_steps` steps and every other robot has been heard at least once."""
        return self.quiet >= self.quiet_steps and self.heard_everyone()

    @property
    def path(self):
        return list(self.agent.path)


def to_message(msg):
    """Plain lists for a ROS message."""
    return {'sender_id': msg.sender_id, 'winning_bid': list(msg.winning_bid_list),
            'winning_agent': list(msg.winning_agent_list), 'timestamps': list(msg.timestamp_list)}


def from_fields(sender_id, winning_bid, winning_agent, timestamps):
    return Message(int(sender_id), [float(v) for v in winning_bid],
                   [int(v) for v in winning_agent], [int(v) for v in timestamps])
