import rclpy
from knowledge_graph_interfaces.srv import (
    AddEdge,
    AddNode,
    UpdateNode,
    DumpGraph,
    NavRoute, 
    Operate,
    Verify
    )
from rclpy.node import Node
import networkx as nx
import knowledge_graph_server.nxgraph_util as nxutil
import matplotlib.pyplot as plt


class KnowledgeGraphServer(Node):
    
    kg = nx.DiGraph()
    floor_attr  = ['level']
    room_attr   = ['class', 'shape', 'size']
    object_attr = ['class', 'color', 'material', 'weight', 'area']
    person_attr = ['name', 'age', 'gender', 'role']

    def __init__(self):
        super().__init__('knowledge_graph')
        self.srv_edge = self.create_service(AddEdge, 'add_edge', self.add_edge)
        self.srv_node = self.create_service(AddNode, 'add_node', self.add_node)
        self.srv_update = self.create_service(UpdateNode, 'update_node', self.add_node)
        self.srv_dump = self.create_service(DumpGraph, 'dump_graph', self.dump_graph)
        self.srv_route = self.create_service(NavRoute, 'nav_route', self.nav_route)
        self.srv_operate = self.create_service(Operate, 'operate', self.operate)
        self.srv_verify = self.create_service(Verify, 'verify_plan', self.verify_plan)

    def add_edge(self, request, response):
        self.get_logger().info('Adding edge %s -> %s (%s)' % (request.parent, request.child, request.relationship))
        
        try:
            self.kg.add_edge(request.parent, request.child,
                            relationship=request.relationship,
                            hierarchy='parent') # hierarchu not necessary in a DiGraph
            response.success = True
        except nx.NetworkXError:
            self.get_logger().error('NetworkX error adding edge')
            response.success = False
    
        return response
    
    def add_node(self, request, response):
        self.get_logger().info('Adding node %s (type: %s)' % (request.id, request.type))
        
        node_type = request.type

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
            attr = request.attr
            attr.append(request.aff)

            node_attr = dict(zip(data, request.attr))
            try:
                self.kg.add_node(request.id, attr=node_attr, affordances=request.aff, status=request.status)
                response.success = True
            except nx.NetworkXError:
                self.get_logger().error('NetworkX error adding node %s' % request.id)
                response.success = False
        else:
            self.get_logger().error('Wrong attributes provided')
            response.success = False
        
        return response
    
    def update_node(self, request, response):
        self.get_logger().info('Updating node %s (type: %s)' % (request.id, request.type))

        node = self.kg.nodes[request.attr[0]]
        
        node_type = request.attr[1]

        if node_type != self.kg.nodes[node]['type']:
            self.get_logger().error('Type mismatch')
            response.success = False
        else:
            attr = request.attr
            attr.append(request.aff)
            
            try:
                self.kg.nodes[node].update(attr)
                response.success = True
            except nx.NetworkXError:
                self.get_logger().error('NetworkX error updating node %s' % node['id'])
                response.success = False
        
            
    def dump_graph(self, request, response):
        self.get_logger().info('Dumping graph in %s format' % request.format.upper())

        try:
            response.success = True
            kg_str = nxutil.graph_to_str(self.kg, request.format)
            if kg_str is None:
                self.get_logger().error('Unknown format %s' % request.format)
                kg_str = ''
                response.success = False
        except Exception as e:
            self.get_logger().error('Error dumping graph: %r' % (e,))
            kg_str = ''
        
        response.kg_str = kg_str
        return response
    
    
    def nav_route(self, request, response):
        self.get_logger().info('Finding route between %s and %s' % (request.origin, request.destination))

        try:
            response.route = nxutil.get_shortest_route(self.kg, request.origin, request.destination)
        except nx.NetworkXNoPath:
            response.route = ''
            response.success = False
            self.get_logger().error('No route found between %s and %s' % (request.origin, request.destination))
            
        return response
    
    def operate(self, request, response):
        self.get_logger().info('Performing operation %s' % request.operation)

        if request.operation == 'collapse': # collapses the graph around the specified node
            root = request.payload[0]
            subgraph = nxutil.collapse_graph(root, self.kg)
            if subgraph is not None:
                subgraph_str = nxutil.graph_to_str(subgraph, request.format)
                response.success = True
            else:
                subgraph_str = None
        elif request.operation == 'expand': # expands level below the specified node
            node = request.payload[0]
            subgraph_str = request.payload[1]
            subgraph = nxutil.str_to_graph(subgraph_str, request.format)
            if subgraph is not None:
                subgraph = nxutil.expand_graph(node, self.kg, subgraph)
                subgraph_str = nxutil.graph_to_str(subgraph, request.format)
            else:
                subgraph_str = None
        elif request.operation == 'contract': # contracts level below the specified node
            node = request.payload[0]
            subgraph_str = request.payload[1]
            subgraph = nxutil.str_to_graph(subgraph_str, request.format)
            if subgraph is not None:
                subgraph = nxutil.contract_graph(node, subgraph)
                subgraph_str = nxutil.graph_to_str(subgraph, request.format)
            else:
                subgraph_str = None
            response.success = True
        else:
            self.get_logger().error('Unknown operation %s' % request.operation)
            subgraph_str = ''
            response.success = False
        
        if subgraph_str is None:
            self.get_logger().error('Impossible to perform operation %s' % request.operation)
            subgraph_str = ''
            response.success = False
        
        response.kg_str = subgraph_str
        return response

    def verify_plan(self, request, response):
        self.get_logger().info('Verifying plan')

        try:
            ret = nxutil.verify_plan(self.kg, request.start_location, request.plan)
            response.feasible = ret[0]
            response.message = ret[1]
        except Exception as e:
            self.get_logger().error('Error verifying plan: %r' % (e,))
            response.feasible = False
        
        self.get_logger().info('Plan verification result: %s' % response.message)
        
        return response

def main(args=None):
    rclpy.init(args=args)

    kg_service = KnowledgeGraphServer()

    rclpy.spin(kg_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()