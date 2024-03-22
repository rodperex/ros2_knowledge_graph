import networkx as nx
import json
import yaml

def dump_json(graph):
    return json.dumps(nx.readwrite.json_graph.adjacency_data(graph))

def dump_yaml(graph):
    return yaml.dump(nx.node_link_data(graph))

def graph_to_str(graph, format):
    if format == 'json':
        return dump_json(graph)
    elif format == 'yaml':
        return dump_yaml(graph)
    else:
        return None
    
def str_to_graph(graph_str, format):
    if format == 'json':
        data = json.loads(graph_str)
        return nx.readwrite.json_graph.adjacency_graph(data)
    elif format == 'yaml':
        data = yaml.safe_load(graph_str)
        return nx.readwrite.yaml.graph_from_dict(data)
    else:
        return None
    
def collapse_graph(root, graph):
    if root not in graph:
        raise ValueError("Node not found in the graph.")

    try:
        subgraph_nodes = list(graph.neighbors(root))
        subgraph_nodes.append(root)  # Add the root node to the list of neighbors
        subgraph = graph.subgraph(subgraph_nodes)
    except nx.NetworkXError:
        subgraph = None
    return subgraph

# def expand_graph(node, graph, subgraph):
#     if node not in graph:
#         raise ValueError("Node not found in the graph.")

#     subgraph = subgraph.copy()

#     successors_in_subgraph = set(subgraph.successors(node))
#     successors_in_kg = set(graph.successors(node))
#     new_successors = successors_in_kg - successors_in_subgraph
#     subgraph.add_nodes_from(new_successors)
#     for successor in new_successors:
#         if successor not in subgraph:
#             subgraph.add_node(successor)
#         for edge in graph.edges(node, data=True):
#             if edge[1] == successor:
#                 # edge[1] are the successors of the node
#                 # edge[2] are the attributes of the edge
#                 subgraph.add_edge(*edge[:2], **edge[2])
#     return subgraph

def expand_graph(node, graph, subgraph, levels=1):
    if node not in graph:
        raise ValueError("Node not found in the graph.")
    
    expanded_subgraph = subgraph.copy()
    
    if levels == 'all':
        max_levels = float('inf')
    else:
        max_levels = int(levels)
    
    def expand_node(node, graph, subgraph, current_level):
        if current_level >= max_levels:
            return subgraph
        
        successors_in_subgraph = set(subgraph.successors(node))
        successors_in_kg = set(graph.successors(node))
        
        new_successors = successors_in_kg - successors_in_subgraph
        
        subgraph.add_nodes_from(new_successors)
        
        for successor in new_successors:
            if successor not in subgraph:
                subgraph.add_node(successor)
            for edge in graph.edges(node, data=True):
                if edge[1] == successor:
                    subgraph.add_edge(*edge[:2], **edge[2])
                    expand_node(successor, graph, subgraph, current_level + 1)
    
    expand_node(node, graph, expanded_subgraph, 0)
    
    return expanded_subgraph


def contract_graph(node, graph):
    if node not in graph:
        raise ValueError("Node not found in the graph.")
    
    contracted_subgraph = graph.copy()

    descendants = list(contracted_subgraph.successors(node))
    for descendant in descendants:
        for successor in contracted_subgraph.successors(descendant):
            contracted_subgraph.add_edge(node, successor, **contracted_subgraph[descendant][successor])
        contracted_subgraph.remove_node(descendant)
    return contracted_subgraph


def transform_to_undirected(digraph):
    undirected_graph = nx.Graph()
    undirected_graph.add_nodes_from(digraph.nodes)

    for edge in digraph.edges:
        undirected_graph.add_edge(edge[0], edge[1])
        undirected_graph.add_edge(edge[1], edge[0])

    return undirected_graph


def get_shortest_route(graph, origin, destination):
    if origin not in graph or destination not in graph:
        raise ValueError("Node not found in the graph.")

    predecessors_origin = set([pred for pred in graph.predecessors(origin) if graph.nodes[pred]['type'] == 'floor'])
    predecessors_destination = set([pred for pred in graph.predecessors(destination) if graph.nodes[pred]['type'] == 'floor'])

    if predecessors_origin & predecessors_destination:
        shortest_path = [origin, destination]
    else:
        undirected_graph = transform_to_undirected(graph)
        shortest_path = nx.shortest_path(undirected_graph, origin, destination)
        for predecessor in predecessors_origin:
            if predecessor in shortest_path:
                shortest_path.remove(predecessor)
                break
        
    return shortest_path
    
    