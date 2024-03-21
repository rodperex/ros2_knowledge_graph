import rclpy
from knowledge_graph_interfaces.srv import AddEdge, AddNode, DumpGraph
from rclpy.node import Node
import networkx as nx
import json
import yaml



class KnowledgeGraphServer(Node):
    
    kg = nx.Graph()

    def __init__(self):
        super().__init__('knowledge_graph')
        self.srv_edge = self.create_service(AddEdge, 'add_edge', self.add_edge)
        self.srv_node = self.create_service(AddNode, 'add_node', self.add_node)
        self.srv_dump = self.create_service(DumpGraph, 'dump_graph', self.dump_graph)

    def add_edge(self, request, response):
        parent = request.parent
        child = request.child
        relationship = request.relationship
        print(f"Adding edge between {parent} and {child} with relationship {relationship}")
        response.success = True
        return response
    
    def add_node(self, request, response):
        response.success = True
        return True

    def dump_graph(self, request, response):
        if request.format == 'json':
            kg_json = nx.readwrite.json_graph.adjacency_data(self.kg)
            kg_str = json.dumps(kg_json)
        elif request.format == 'yaml':
            kg_data = nx.node_link_data(self.kg)
            kg_str = yaml.dump(kg_data)
        
        response.kg_str = kg_str
        return response


def main(args=None):
    rclpy.init(args=args)

    kg_service = KnowledgeGraphServer()

    rclpy.spin(kg_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()