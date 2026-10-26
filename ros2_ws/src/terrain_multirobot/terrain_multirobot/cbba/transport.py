# g[i][i] = 1 but will be ignored
def deliver_message(messages, graph):
    inboxes = {i: [] for i in range(len(graph))}

    for k in messages:
        if k is None:
            continue
        for i, val in enumerate(graph[k.sender_id]):
            if val == 1 and i != k.sender_id:
                inboxes[i].append(k)

    return inboxes

