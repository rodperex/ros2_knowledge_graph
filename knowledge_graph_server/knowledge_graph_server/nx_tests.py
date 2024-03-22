import networkx as nx
import matplotlib.pyplot as plt
import nxgraph_util as nxutil


scene_graph = nx.DiGraph()

floor_attr  = ['id', 'type', 'level']
room_attr   = ['id', 'type', 'class', 'shape', 'size']
object_attr = ['id', 'type', 'class', 'color', 'material', 'weight', 'area', 'affordances']
person_attr = ['id', 'type', 'name', 'age', 'gender', 'role']


floor = dict(zip(floor_attr, ['first_floor', 'floor', '1']))
scene_graph.add_node(floor['id'], **floor)
floor = dict(zip(floor_attr, ['ground_floor', 'floor', '0']))
scene_graph.add_node(floor['id'], **floor)

scene_graph.add_edge(floor['id'], 'first_floor' , relationship='connects')

room = dict(zip(room_attr, ['kitchen_1', 'room', 'kitchen', 'square', 'big']))
scene_graph.add_node(room['id'], **room)
scene_graph.add_edge('ground_floor', room['id'], relationship='contains')

room = dict(zip(room_attr, ['living_room_1', 'room', 'living_room', 'square', 'big']))
scene_graph.add_node(room['id'], **room)
scene_graph.add_edge('ground_floor', room['id'], relationship='contains')

object = dict(zip(object_attr, ['table_1', 'object', 'table', 'brown', 'wood', 'heavy', 'big', ['put on', 'pick from']]))
scene_graph.add_node(object['id'], **object)
scene_graph.add_edge('living_room_1', object['id'], relationship='contains')

object = dict(zip(object_attr, ['chair_1', 'object', 'chair', 'brown', 'wood', 'light', 'small', ['sit']]))
scene_graph.add_node(object['id'], **object)
scene_graph.add_edge('living_room_1', object['id'], relationship='contains')

object = dict(zip(object_attr, ['chair_2', 'object', 'chair', 'brown', 'wood', 'light', 'small', ['sit']]))
scene_graph.add_node(object['id'], **object)
scene_graph.add_edge('kitchen_1', object['id'], relationship='contains')

object = dict(zip(object_attr, ['fridge_1', 'object', 'fridge', 'white', 'metal', 'heavy', 'big', ['open', 'close']]))
scene_graph.add_node(object['id'], **object)
scene_graph.add_edge('kitchen_1', object['id'], relationship='contains')

person = dict(zip(person_attr, ['John_1', 'person', 'John', '30', 'male', 'owner']))
scene_graph.add_node(person['id'], **person)
scene_graph.add_edge('living_room_1', person['id'], relationship='contains')
scene_graph.add_edge(person['id'], 'table_1', relationship='looks_at')

person = dict(zip(person_attr, ['Mary_1', 'person', 'Mary', '56', 'female', 'guest']))
scene_graph.add_node(person['id'], **person)
scene_graph.add_edge('kitchen_1', person['id'], relationship='contains')
scene_graph.add_edge(person['id'], 'chair_2', relationship='sits_on')

room = dict(zip(room_attr, ['bedroom_1', 'room', 'bedroom', 'square', 'small']))
scene_graph.add_node(room['id'], **room)
scene_graph.add_edge('first_floor', room['id'], relationship='contains')

object = dict(zip(object_attr, ['bed_1', 'object', 'bed', 'blue', 'wood', 'heavy', 'medium', ['make']]))
scene_graph.add_node(object['id'], **object)
scene_graph.add_edge('bedroom_1', object['id'], relationship='contains')

person = dict(zip(person_attr, ['Magic_1', 'person', 'Magic', '33', 'male', 'guest']))
scene_graph.add_node(person['id'], **person)
scene_graph.add_edge('bedroom_1', person['id'], relationship='contains')
scene_graph.add_edge(person['id'], 'bed_1', relationship='lays_on')


print('Shortest route between living_room_1 and kitchen_1:', nxutil.get_shortest_route(scene_graph, 'living_room_1', 'kitchen_1'))
print('Shortest route between living_room_1 and bedroom_1:', nxutil.get_shortest_route(scene_graph, 'living_room_1', 'bedroom_1'))

plt.figure(1)
plt.figure(2)
plt.figure(3)
plt.figure(4)

pos = nx.spring_layout(scene_graph)
plt.figure(1)
plt.title('Scene Graph')
nx.draw(scene_graph, pos, with_labels=True, font_weight='bold')
edge_labels = nx.get_edge_attributes(scene_graph, 'relationship')
nx.draw_networkx_edge_labels(scene_graph, pos, edge_labels=edge_labels)

subgraph = nxutil.collapse_graph('ground_floor', scene_graph)
pos = nx.spring_layout(subgraph)
plt.figure(2)
plt.title('Collapsed around ground_floor')
nx.draw(subgraph, pos, with_labels=True, font_weight='bold')
edge_labels = nx.get_edge_attributes(subgraph, 'relationship')
nx.draw_networkx_edge_labels(subgraph, pos, edge_labels=edge_labels)

subgraph = nxutil.expand_graph('living_room_1', scene_graph, subgraph, levels=1)
pos = nx.spring_layout(subgraph)
plt.figure(3)
plt.title('Expanded around living_room_1')
nx.draw(subgraph, pos, with_labels=True, font_weight='bold')
edge_labels = nx.get_edge_attributes(subgraph, 'relationship')
nx.draw_networkx_edge_labels(subgraph, pos, edge_labels=edge_labels)

subgraph = nxutil.contract_graph('living_room_1', subgraph)
pos = nx.spring_layout(subgraph)
plt.figure(4)
plt.title('Contracted around living_room_1')
nx.draw(subgraph, pos, with_labels=True, font_weight='bold')
edge_labels = nx.get_edge_attributes(subgraph, 'relationship')
nx.draw_networkx_edge_labels(subgraph, pos, edge_labels=edge_labels)

plt.ion()
plt.show()
input("Press Enter to close all windows and end...")

plt.close('all')


