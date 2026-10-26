"""Lock-step CBBA: every agent bids, then every agent receives the others'
messages, then every agent resolves conflicts. One pass = one round.
This is the same loop as the offline tests and cbba/main.py, as a reusable
function that also records what every agent believed in every round."""
from .bundle import build_bundle
from .consensus import consume_message, produce_message, release
from .scoring import agent_score
from .search_algo import bfs
from .transport import deliver_message


def snapshot(agent):
    return {'bundle': list(agent.bundle),
            'path': list(agent.path),
            'z': list(agent.winning_agent_list),
            'y': list(agent.winning_bid_list)}


def simulate(agents, tasks, max_bundle, graph=None, max_rounds=100):
    """agents: {id: Agent} with ids 0..N-1; tasks: {id: Task} with ids 0..M-1;
    graph: adjacency matrix (None = everyone hears everyone).

    Stops once nothing has changed for D rounds in a row, D = network depth:
    information needs up to D rounds to cross the network, so one quiet round
    is not enough to call it converged.
    Returns {'rounds': [...], 'converged': bool, 'scores': {id: points}}."""
    n = len(agents)
    if graph is None:
        graph = [[1] * n for _ in range(n)]
    disconnected, depth = bfs(n, graph)
    quiet_needed = max(depth, 1)

    rounds, quiet = [], 0
    for rnd in range(1, max_rounds + 1):
        before = [snapshot(agents[i]) for i in range(n)]

        outgoing = [None] * n
        for i in range(n):
            build_bundle(agents[i], tasks, max_bundle)
            outgoing[i] = produce_message(agents[i], rnd)
        after_bidding = [snapshot(agents[i]) for i in range(n)]

        for i, msgs in deliver_message(outgoing, graph).items():
            for m in msgs:
                consume_message(agents[i], m)
            release(agents[i])

        rounds.append({'bids': after_bidding,
                       'agreed': [snapshot(agents[i]) for i in range(n)]})
        quiet = quiet + 1 if rounds[-1]['agreed'] == before else 0
        if quiet >= quiet_needed:
            break

    return {'rounds': rounds,
            'converged': quiet >= quiet_needed,
            'disconnected': disconnected,
            'scores': {i: agent_score(agents[i], tasks) for i in range(n)}}
