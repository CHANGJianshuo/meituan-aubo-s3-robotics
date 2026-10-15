"""Graph source/type/conflict evidence; no server launch or FSM writes."""

def graph_snapshot(node, namespace):
    from rclpy.action.graph import get_action_server_names_and_types_by_node, get_action_client_names_and_types_by_node
    ns = namespace.rstrip('/')
    expected = {ns + '/task/execute': (ns + '/mtc_mock_task', 'mtc_interfaces/action/ExecuteTask'),
                ns + '/manipulation/pick_place': (ns + '/mtc_mock_pick_place', 'mtc_interfaces/action/PickPlace')}
    nodes = node.get_node_names_and_namespaces()
    servers, clients, issues = [], [], []
    for name, space in nodes:
        full = space.rstrip('/') + '/' + name
        for action, types in get_action_server_names_and_types_by_node(node, name, space):
            servers.append(dict(node=full, action=action, types=types))
        for action, types in get_action_client_names_and_types_by_node(node, name, space):
            clients.append(dict(node=full, action=action, types=types))
        if name in ('task_executor', 'pick_place_server', 'mtc_task_executor', 'mtc_pick_place_server'):
            issues.append('Legacy Python server node present: ' + full)
    for action, (owner, msgtype) in expected.items():
        matches = [s for s in servers if s['action'] == action]
        if len(matches) != 1 or matches[0]['node'] != owner or matches[0]['types'] != [msgtype]:
            issues.append('Missing, conflicting or wrong source/type: ' + action)
    for s in servers:
        if s['action'] in ('/mtc/task/execute', '/mtc/manipulation/pick_place'):
            issues.append('Legacy canonical Action server still present: ' + s['node'])
    for owner, _ in expected.values():
        if sum(space.rstrip('/') + '/' + name == owner for name, space in nodes) != 1:
            issues.append('Missing or duplicate node: ' + owner)
    child_clients = [s for s in clients if s['action'] == ns + '/manipulation/pick_place']
    if not any(s['node'] == ns + '/mtc_mock_task' and s['types'] == ['mtc_interfaces/action/PickPlace'] for s in child_clients):
        issues.append('C++ Task child ActionClient missing')
    topics = []
    for topic, types in node.get_topic_names_and_types():
        if not topic.startswith(ns + '/'):
            continue
        entries = []
        for info in node.get_publishers_info_by_topic(topic):
            qos = info.qos_profile
            entries.append(dict(node=info.node_namespace.rstrip('/') + '/' + info.node_name,
                gid=bytes(info.endpoint_gid).hex(), type=info.topic_type,
                qos=dict(reliability=str(qos.reliability), durability=str(qos.durability),
                         history=str(qos.history), depth=qos.depth)))
        topics.append(dict(name=topic, types=types, publishers=entries))
    return dict(valid=not issues, issues=issues,
                nodes=[space.rstrip('/') + '/' + name for name, space in nodes],
                servers=servers, clients=clients, topics=topics,
                task_servers=[s['node'] for s in servers if s['action'] == ns + '/task/execute'])
