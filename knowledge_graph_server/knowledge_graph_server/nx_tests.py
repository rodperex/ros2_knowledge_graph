import networkx as nx
import matplotlib.pyplot as plt
import nxgraph_util as nxutil


scene_graph = nx.DiGraph()

floor_attr  = ['level']
room_attr   = ['class', 'shape', 'size']
object_attr = ['class', 'color', 'material', 'weight', 'area']
person_attr = ['name', 'age', 'gender', 'role']

scene_graph.add_node('first_floor', type='floor', attr=dict(zip(floor_attr, ['1'])), affordances=[], status='')
scene_graph.add_node('ground_floor', type='floor', attr=dict(zip(floor_attr, ['0'])), affordances=[], status='')
scene_graph.add_edge('ground_floor', 'first_floor' , relationship='connects')

scene_graph.add_node('kitchen_1', type='room', attr=dict(zip(room_attr, ['kitchen', 'square', 'big'])), affordances=[], status='')
scene_graph.add_edge('ground_floor', 'kitchen_1', relationship='contains')

scene_graph.add_node('living_room_1', type='room', attr=dict(zip(room_attr, ['living_room', 'square', 'big'])), affordances=[], status='')
scene_graph.add_edge('ground_floor', 'living_room_1', relationship='contains')

scene_graph.add_node('table_1', type='object', attr=dict(zip(object_attr, ['table', 'brown', 'wood', 'heavy', 'big'])), affordances=['put on', 'pick from'], status='')
scene_graph.add_edge('living_room_1', 'table_1', relationship='contains')

scene_graph.add_node('chair_1', type='object', attr=dict(zip(object_attr, ['chair', 'brown', 'wood', 'light', 'small'])), affordances=['sit'], status='free')
scene_graph.add_edge('living_room_1', 'chair_1', relationship='contains')

scene_graph.add_node('chair_2', type='object', attr=dict(zip(object_attr, ['chair', 'brown', 'wood', 'light', 'small'])), affordances=['sit'], status='free')
scene_graph.add_edge('kitchen_1', 'chair_2', relationship='contains')
node_data = scene_graph.nodes['chair_2']
node_data['status'] = 'busy'
node_data['affordances'].append('destroy')
scene_graph.nodes['chair_2'].update(node_data)

scene_graph.add_node('fridge_1', type='object', attr=dict(zip(object_attr, ['chair', 'brown', 'wood', 'light', 'small'])), affordances=['open', 'close'], status='closed')
scene_graph.add_edge('kitchen_1', 'fridge_1', relationship='contains')

scene_graph.add_node('John_1', type='person', attr=dict(zip(person_attr, ['John', '30', 'male', 'owner'])), affordances=['talk'], status='')
scene_graph.add_edge('living_room_1', 'John_1', relationship='contains')
scene_graph.add_edge('John_1', 'table_1', relationship='looks_at')

scene_graph.add_node('Mary_1', type='person', attr=dict(zip(person_attr, ['Mary', '56', 'female', 'guest'])), affordances=['talk'], status='')
scene_graph.add_edge('kitchen_1', 'Mary_1', relationship='contains')
scene_graph.add_edge('Mary_1', 'chair_2', relationship='sits_on')

scene_graph.add_node('bedroom_1', type='room', attr=dict(zip(room_attr, ['bedroom', 'square', 'small'])), affordances=[], status='')
scene_graph.add_edge('first_floor', 'bedroom_1', relationship='contains')

scene_graph.add_node('bed_1', type='object', attr=dict(zip(object_attr, ['bed', 'blue', 'wood', 'heavy', 'medium'])), affordances=['make'], status='busy')
scene_graph.add_edge('bedroom_1', 'bed_1', relationship='contains')

scene_graph.add_node('Magic_1', type='person', attr=dict(zip(person_attr, ['Magic', '33', 'male', 'guest'])), affordances=[], status='')
scene_graph.add_edge('bedroom_1', 'Magic_1', relationship='contains')
scene_graph.add_edge('Magic_1', 'bed_1', relationship='lays_on')


print('Shortest route between living_room_1 and kitchen_1:', nxutil.get_shortest_route(scene_graph, 'living_room_1', 'kitchen_1'))
print('Shortest route between living_room_1 and bedroom_1:', nxutil.get_shortest_route(scene_graph, 'living_room_1', 'bedroom_1'))
print('The bed \'bed_1\' is :', scene_graph.nodes['bed_1']['status'])

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


