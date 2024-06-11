import rclpy
from knowledge_graph_interfaces.srv import (
    AddEdge,
    RemoveEdge,
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

class KnowledgeGraphServer(Node):
    
    __kg = nx.DiGraph()
    __working_graph = __kg
    __floor_attr  = ['level']
    __room_attr   = ['class', 'shape', 'size']
    __object_attr = ['class', 'color', 'material', 'weight', 'size']
    __person_attr = ['name', 'age', 'gender', 'role']

    def __init__(self):
        super().__init__('knowledge_graph')
        self.srv_edge = self.create_service(
            AddEdge, 'add_edge', self.__add_edge)
        self.srv_remove_edge = self.create_service(
            RemoveEdge, 'remove_edge', self.__remove_edge)
        self.srv_node = self.create_service(
            AddNode, 'add_node', self.__add_node)
        self.srv_update_node = self.create_service(
            UpdateNode, 'update_node', self.__update_node)
        self.srv_dump = self.create_service(
            DumpGraph, 'dump_graph', self.__dump_kg)
        self.srv_route = self.create_service(
            NavRoute, 'nav_route', self.__nav_route)
        self.srv_operate = self.create_service(
            Operate, 'operate', self.__operate)
        self.srv_verify = self.create_service(
            Verify, 'verify_plan', self.__verify_plan)

    def __add_edge(self, request, response):
        self.get_logger().info('Adding edge %s -> %s (%s)' % (request.parent, request.child, request.relationship))
        
        try:
            self.__kg.add_edge(request.parent, request.child,
                            relationship=request.relationship,
                            hierarchy='parent') # hierarchy not necessary in a DiGraph
            response.success = True
            self.__working_graph = self.__kg
        except nx.NetworkXError:
            self.get_logger().error('NetworkX error adding edge')
            response.success = False
    
        return response
    
    def __remove_edge(self, request, response):
        self.get_logger().info('Removing edge %s -> %s' % (request.parent, request.child))

        try:
            self.__kg.remove_edge(request.parent, request.child)
            response.success = True
            self.__working_graph = self.__kg
        except nx.NetworkXError:
            self.get_logger().error('NetworkX error removing edge')
            response.success = False
        
        return response
    
    def __add_node(self, request, response):
        self.get_logger().info('Adding node %s (type: %s)' % (request.id, request.type))
        
        node_type = request.type

        if node_type == 'floor':
            data = self.__floor_attr
        elif node_type == 'room':
            data = self.__room_attr
        elif node_type == 'object':
            data = self.__object_attr
        elif node_type == 'person':
            data = self.__person_attr
        elif node_type == 'robot':
            data = []
        else:
            data = None

        if data is not None:
            node_attr = dict(zip(data, request.attr))
            try:
                self.__kg.add_node(request.id, type=request.type, attr=node_attr, affordances=request.aff, status=request.status)
                response.success = True
                self.__working_graph = self.__kg
            except nx.NetworkXError:
                self.get_logger().error('NetworkX error adding node %s' % request.id)
                response.success = False
        else:
            self.get_logger().error('Wrong attributes provided')
            response.success = False
        
        return response
    
    def __update_node(self, request, response):
        self.get_logger().info('Updating node %s (type: %s)' % (request.id, request.type))

        node_data = self.__kg.nodes[request.id]
        node_type = request.type

        if node_type != node_data['type']:
            self.get_logger().error('Type mismatch')
            response.success = False
        else:
            if node_type == 'floor':
                data = self.__floor_attr
            elif node_type == 'room':
                data = self.__room_attr
            elif node_type == 'object':
                data = self.__object_attr
            elif node_type == 'person':
                data = self.__person_attr
            elif node_type == 'robot':
                data = []
            else:
                data = None
            
            node_data['status'] = request.status
            node_data['affordances'] = request.aff
            node_data['attr'] = dict(zip(data, request.attr))
    
            try:
                self.__kg.nodes[request.id].update(node_data)
                self.get_logger().info('Node %s updated' % request.id)
                response.success = True
                self.__working_graph = self.__kg
            except nx.NetworkXError:
                self.get_logger().error('NetworkX error updating node %s' % request.id)
                response.success = False
        return response
        
            
    def __dump_kg(self, request, response):
    
        try:
            if request.full:
                self.get_logger().info('Dumping knowledge graph in %s format' % request.format.upper())
                graph = self.__kg
            else:
                self.get_logger().info('Dumping working graph in %s format' % request.format.upper())
                graph = self.__working_graph
            response.success = True
            kg_str = self.__dump_nxgraph(graph, request.format)
            if kg_str is None:
                self.get_logger().error('Unknown format %s' % request.format)
                kg_str = ''
                response.success = False
        except Exception as e:
            self.get_logger().error('Error dumping graph: %r' % (e,))
            kg_str = ''
        
        response.kg_str = kg_str
        return response
    
    def __dump_nxgraph(self, graph, format):
        self.get_logger().info('Dumping graph in %s format' % format.upper())

        g_str = nxutil.graph_to_str(graph, format)
        if g_str is None:
            g_str = ''
              
        return g_str
    
    def __nav_route(self, request, response):
        self.get_logger().info('Finding route between %s and %s' % (request.origin, request.destination))

        try:
            response.route = nxutil.get_shortest_route(self.__kg, request.origin, request.destination)
            response.success = True
        except nx.NetworkXNoPath:
            response.route = ''
            response.success = False
            self.get_logger().error('No route found between %s and %s' % (request.origin, request.destination))
            
        return response
    
    def __operate(self, request, response):
        self.get_logger().info('Operation request: %s' % request.operation)

        try:
            if request.operation == 'collapse': # collapses the graph around the specified node
                root = request.payload[0]
                self.get_logger().info('Collapsing around %s' % root)
                self.get_logger().debug('KG nodes: %s' % self.__kg.nodes)
                self.get_logger().info('Working graph nodes: %s' % self.__working_graph.nodes)
                subgraph = nxutil.collapse_graph(root, self.__working_graph)
                if subgraph is not None:
                    self.get_logger().info('Subgraph nodes: %s' % subgraph.nodes)
                    subgraph_str = nxutil.graph_to_str(subgraph, request.format)
                    response.success = True
                    self.__working_graph = subgraph
                else:
                    subgraph_str = None
            elif request.operation == 'expand': # expands level below the specified node
                node = request.payload[0]
                self.get_logger().info('Expanding around %s' % node)
                self.get_logger().debug('KG nodes: %s' % self.__kg.nodes)
                self.get_logger().info('Working graph nodes: %s' % self.__working_graph.nodes)
                subgraph = nxutil.expand_graph(node, self.__kg, self.__working_graph,
                                               levels=int(request.payload[1]))
                if subgraph is not None:
                    self.get_logger().info('Subgraph nodes: %s' % subgraph.nodes)
                    subgraph_str = nxutil.graph_to_str(subgraph, request.format)
                    self.__working_graph = subgraph
                else:
                    subgraph_str = None
            elif request.operation == 'contract': # contracts level below the specified node
                node = request.payload[0]
                self.get_logger().info('Contracting %s' % node)
                self.get_logger().debug('KG nodes: %s' % self.__kg.nodes)
                self.get_logger().info('Working graph nodes: %s' % self.__working_graph.nodes)
                if subgraph is not None:
                    self.get_logger().info('Subgraph nodes: %s' % subgraph.nodes)
                    subgraph = nxutil.contract_graph(node, self.__working_graph)
                    subgraph_str = nxutil.graph_to_str(subgraph, request.format)
                    self.__working_graph = subgraph
                else:
                    subgraph_str = None
                response.success = True
            else:
                self.get_logger().error('Unknown operation %s' % request.operation)
                subgraph_str = ''
                response.success = False
            
            if subgraph_str is None:
                self.get_logger().error('Impossible to perform operation \'%s\'' % request.operation)
                subgraph_str = ''
                response.success = False
            
            self.get_logger().info('Retrieving \'%s\' result in %s format' % (request.operation, request.format.upper()))
            response.kg_str = subgraph_str
            self.get_logger().debug('Operation result: %s' % response.kg_str)
            return response
        except Exception as e:
            self.get_logger().error('Error performing operation: %r' % (e,))
            response.success = False
            response.kg_str = ''
            return response

    def __verify_plan(self, request, response):
        self.get_logger().info('Verifying plan')

        try:
            ret = nxutil.verify_plan(self.__kg, request.robot_id, request.plan)
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