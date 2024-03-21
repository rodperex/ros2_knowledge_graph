import rclpy
from knowledge_graph_interfaces.srv import AddEdge, AddNode, DumpGraph
from rclpy.node import Node
import networkx as nx
import json
import yaml



class KnowledgeGraphServer(Node):
    
    kg = nx.Graph()
    floor_attr  = ['id', 'type', 'level']
    room_attr   = ['id', 'type', 'class', 'shape', 'size']
    object_attr = ['id', 'type', 'class', 'color', 'material', 'weight', 'area', 'affordances']
    person_attr = ['id', 'type', 'name', 'age', 'gender', 'role']

    def __init__(self):
        super().__init__('knowledge_graph')
        self.srv_edge = self.create_service(AddEdge, 'add_edge', self.add_edge)
        self.srv_node = self.create_service(AddNode, 'add_node', self.add_node)
        self.srv_dump = self.create_service(DumpGraph, 'dump_graph', self.dump_graph)

    def add_edge(self, request, response):
        self.get_logger().info('Adding edge between %s and %s (%s)' % (request.parent, request.child, request.relationship))
        
        try:
            self.kg.add_edge(request.parent, request.child, relationship=request.relationship)
            response.success = True
        except nx.NetworkXError:
            self.get_logger().error('NetworkX error adding edge')
            response.success = False
    
        return response
    
    def add_node(self, request, response):
        self.get_logger().info('Adding node %s of type %s' % (request.attr[0], request.attr[1]))
        
        node_type = request.attr[1]

        if node_type == 'floor':
            data = self.floor_attr
        elif node_type == 'room':
            data = self.room_attr
        elif node_type == 'object':
            data = self.object_attr
        elif node_type == 'person':
            data = self.person_attr
        else:
            data = None

        if data is not None:
            node = dict(zip(data, request.attr))
            try:
                self.kg.add_node(node['id'], **node)
                response.success = True
            except nx.NetworkXError:
                self.get_logger().error('NetworkX error adding node %s' % node['id'])
                response.success = False
        else:
            self.get_logger().error('Wrong attributes provided')
            response.success = False
        
        return response

    def dump_graph(self, request, response):
        self.get_logger().info('Dumping graph in %s format' % request.format.upper())

        try:
            response.success = True
            if request.format == 'json':
                kg_json = nx.readwrite.json_graph.adjacency_data(self.kg)
                kg_str = json.dumps(kg_json)
            elif request.format == 'yaml':
                kg_data = nx.node_link_data(self.kg)
                kg_str = yaml.dump(kg_data)
            else:
                self.get_logger().error('Unknown format %s' % request.format)
                response.success = False
                kg_str = ''
        except Exception as e:
            self.get_logger().error('Error dumping graph: %r' % (e,))
            kg_str = ''
        
        response.kg_str = kg_str
        return response


def main(args=None):
    rclpy.init(args=args)

    kg_service = KnowledgeGraphServer()

    rclpy.spin(kg_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()