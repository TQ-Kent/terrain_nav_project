import random

from terrain_multirobot.allocation import RobotAllocator
from terrain_multirobot.cbba.models import Agent, Task
from terrain_multirobot.cbba.scoring import agent_score
from terrain_multirobot.cbba.simulation import simulate


def run_async(positions, tasks, max_bundle, seed, steps=200):
    """Each robot steps on its own turn in a random order; a message reaches everyone before the next step."""
    rng = random.Random(seed)
    robots = [RobotAllocator(i, p, tasks, len(positions), speed=1.0, max_bundle=max_bundle, quiet_steps=8)
              for i, p in enumerate(positions)]
    inboxes = [[] for _ in robots]
    for _ in range(steps):
        order = list(range(len(robots)))
        rng.shuffle(order)
        for i in order:
            inbox, inboxes[i] = inboxes[i], []
            msg = robots[i].step(inbox)
            for k in range(len(robots)):
                if k != i:
                    inboxes[k].append(msg)
        if all(r.converged() for r in robots):
            break
    return robots


def random_case(seed, n_robots, n_tasks):
    rng = random.Random(seed)
    pos = [(rng.uniform(0, 100), rng.uniform(0, 60)) for _ in range(n_robots)]
    tasks = [(rng.uniform(0, 100), rng.uniform(0, 60), rng.uniform(50, 150), 0.98) for _ in range(n_tasks)]
    return pos, tasks


def test_every_task_assigned_once_and_robots_agree():
    for seed in range(20):
        pos, tasks = random_case(seed, 3, 6)
        robots = run_async(pos, tasks, max_bundle=3, seed=seed)
        assert all(r.converged() for r in robots), seed
        owned = [t for r in robots for t in r.path]
        assert sorted(owned) == list(range(6)), (seed, owned)
        # all robots hold the same winner table
        tables = {tuple(r.agent.winning_agent_list) for r in robots}
        assert len(tables) == 1, seed


def test_score_close_to_lockstep_reference():
    worse = 0
    for seed in range(20):
        pos, tasks = random_case(seed, 3, 6)
        robots = run_async(pos, tasks, max_bundle=3, seed=seed)
        mine = sum(agent_score(r.agent, r.tasks) for r in robots)
        agents = {i: Agent(i, p, len(tasks), len(pos), speed=1.0) for i, p in enumerate(pos)}
        td = {j: Task(j, (x, y), v, d) for j, (x, y, v, d) in enumerate(tasks)}
        ref = sum(simulate(agents, td, 3)['scores'].values())
        if mine < 0.95 * ref:
            worse += 1
    assert worse == 0


def test_more_tasks_than_capacity_leaves_extras_unassigned():
    pos, tasks = random_case(1, 2, 8)
    robots = run_async(pos, tasks, max_bundle=2, seed=1)
    owned = [t for r in robots for t in r.path]
    assert len(owned) == len(set(owned)) == 4
