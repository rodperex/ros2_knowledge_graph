import rclpy
from knowledge_graph_interfaces.msg import PlanAction
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
import matplotlib.pyplot as plt
import json
import yaml

class KnowledgeGraphClient(Node):

    def __init__(self):
        super().__init__('kg_client')
        

    def create_verify_client(self):
        self.cli = self.create_client(Verify, 'verify_plan')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service \'verify_plan\' not available, waiting again...')
        self.req = Verify.Request()
    
    def create_add_node_client(self):
        self.cli = self.create_client(AddNode, 'add_node')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service \'add_node\' not available, waiting again...')
        self.req = AddNode.Request()
    
    def create_update_node_client(self):
        self.cli = self.create_client(UpdateNode, 'update_node')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service \'update_node\' not available, waiting again...')
        self.req = UpdateNode.Request()

    def create_add_edge_client(self):
        self.cli = self.create_client(AddEdge, 'add_edge')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service \'add_edge\' not available, waiting again...')
        self.req = AddEdge.Request()
    
    def create_dump_graph_client(self):
        self.cli = self.create_client(DumpGraph, 'dump_graph')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service \'dump_graph\' not available, waiting again...')
        self.req = DumpGraph.Request()

    def send_verify_request(self, start_location, plan):
        self.req.plan = plan
        self.req.start_location = start_location
        self.future = self.cli.call_async(self.req)

    def send_add_node_request(self, id, type, attr, aff=[], status=''):
        self.req.id = id
        self.req.type = type
        self.req.attr = attr
        self.req.aff = aff
        self.req.status = status
        self.future = self.cli.call_async(self.req)
    
    def send_update_node_request(self, id, type, attr, aff=[], status=''):
        self.req.id = id
        self.req.type = type
        self.req.attr = attr
        self.req.aff = aff
        self.req.status = status
        self.future = self.cli.call_async(self.req)

    def send_add_edge_request(self, parent, child, relationship):
        self.req.parent = parent
        self.req.child = child
        self.req.relationship = relationship
        self.future = self.cli.call_async(self.req)

    def send_dump_graph_request(self, format):
        self.req.format = format
        self.future = self.cli.call_async(self.req)

    def get_response(self):
        # Wait for the response
        rclpy.spin_until_future_complete(self, self.future)
        try:
            # Retrieve and return the response
            return self.future.result()
        except Exception as e:
            self.get_logger().error('Service call failed: %r' % (e,))
            return None


def create_sample_graph(client):
    
    client.create_add_node_client()

    attr = ['1']
    client.send_add_node_request('first_floor', 'floor', attr)
    response = client.get_response()
    print('Response received: ' + str(response))

    attr = ['0']
    client.send_add_node_request('ground_floor', 'floor',attr)
    response = client.get_response()
    print('Response received: ' + str(response))

    attr = ['kitchen', 'square', 'big']
    client.send_add_node_request('kitchen_1', 'room', attr)
    response = client.get_response()
    print('Response received: ' + str(response))

    attr = ['living_room', 'square', 'big']
    client.send_add_node_request('living_room_1', 'room', attr)
    response = client.get_response()
    print('Response received: ' + str(response))

    attr = ['table', 'brown', 'wood', 'heavy', 'big']
    client.send_add_node_request('table_1', 'object', attr, ['put on', 'pick from'])
    response = client.get_response()
    print('Response received: ' + str(response))

    attr = ['chair', 'brown', 'wood', 'light', 'small']
    client.send_add_node_request('chair_1', 'object', attr, ['sit', 'pick', 'put'], 'free')
    response = client.get_response()
    print('Response received: ' + str(response))

    attr = ['chair', 'brown', 'wood', 'light', 'small']
    client.send_add_node_request('chair_2', 'object', attr, ['sit'], 'free')
    response = client.get_response()
    print('Response received: ' + str(response))

    attr = ['fridge', 'white', 'metal', 'heavy', 'big']
    client.send_add_node_request('fridge_1', 'object', attr, ['open', 'close'], 'closed')
    response = client.get_response()
    print('Response received: ' + str(response))


    attr = ['John', '30', 'male', 'owner']
    client.send_add_node_request('John_1', 'person', attr, ['talk'])
    response = client.get_response()
    print('Response received: ' + str(response))

    attr = ['Mary', '56', 'female', 'guest']
    client.send_add_node_request('Mary_1', 'person', attr, ['talk'])
    response = client.get_response()
    print('Response received: ' + str(response))

    attr = ['bedroom', 'square', 'small']
    client.send_add_node_request('bedroom_1', 'room', attr)
    response = client.get_response()
    print('Response received: ' + str(response))

    attr = ['bed', 'blue', 'wood', 'heavy', 'medium']
    client.send_add_node_request('bed_1', 'object', attr, ['make'], 'free')
    response = client.get_response()
    print('Response received: ' + str(response))

    attr = ['Magic', '33', 'male', 'guest']
    client.send_add_node_request('Magic_1', 'person', attr)
    response = client.get_response()
    print('Response received: ' + str(response))


    client.create_add_edge_client()
    client.send_add_edge_request('ground_floor', 'first_floor', 'connects')
    response = client.get_response()
    print('Response received: ' + str(response))

    client.send_add_edge_request('first_floor', 'kitchen_1', 'contains')
    response = client.get_response()
    print('Response received: ' + str(response))

    client.send_add_edge_request('first_floor', 'living_room_1', 'contains')
    response = client.get_response()
    print('Response received: ' + str(response))

    client.send_add_edge_request('living_room_1', 'table_1', 'contains')
    response = client.get_response()
    print('Response received: ' + str(response))

    client.send_add_edge_request('living_room_1', 'chair_1', 'contains')
    response = client.get_response()
    print('Response received: ' + str(response))

    client.send_add_edge_request('living_room_1', 'chair_2', 'contains')
    response = client.get_response()
    print('Response received: ' + str(response))

    client.send_add_edge_request('kitchen_1', 'fridge_1', 'contains')
    response = client.get_response()
    print('Response received: ' + str(response))

    client.send_add_edge_request('living_room_1', 'John_1', 'contains')
    response = client.get_response()
    print('Response received: ' + str(response))

    client.send_add_edge_request('living_room_1', 'Mary_1', 'contains')
    response = client.get_response()
    print('Response received: ' + str(response))

    client.send_add_edge_request('first_floor', 'bedroom_1', 'contains')
    response = client.get_response()
    print('Response received: ' + str(response))

    client.send_add_edge_request('bedroom_1', 'bed_1', 'contains')
    response = client.get_response()
    print('Response received: ' + str(response))

    client.send_add_edge_request('bedroom_1', 'Magic_1', 'contains')
    response = client.get_response()
    print('Response received: ' + str(response))


def create_sample_feasible_plan_1():
    plan = [
        PlanAction(action='navigate', target=['kitchen_1', 'living_room_1']),
        PlanAction(action='pick', target=['chair_1']),
        PlanAction(action='navigate', target=['living_room_1', 'kitchen_1']),
        PlanAction(action='put', target=['chair_1']),
        PlanAction(action='talk', target=['Mary_1'])
        ]
    return ['kitchen_1', plan]

def create_sample_feasible_plan_2():
    plan = [
        PlanAction(action='open', target=['fridge_1']),
        PlanAction(action='talk', target=['Mary_1'])
        ]
    return ['kitchen_1', plan]

def print_plan(start_location, plan):
    print('\t* Start location:', start_location)
    for action in plan:
        print('\t-', action.action, ':', action.target)

def create_graph_from_json(json_str):
    json_data = json.loads(json_str)
    
    return nx.json_graph.adjacency_graph(json_data)

def create_graph_from_yaml(yaml_str):
    yaml_data = yaml.safe_load(yaml_str)
    
    return nx.json_graph.node_link_graph(yaml_data)
    
def main(args=None):
    rclpy.init(args=args)
    client = KnowledgeGraphClient()

    create_sample_graph(client)

    client.create_verify_client()

    [start_location, plan] = create_sample_feasible_plan_1()
    print('Plan to verify:')
    print_plan(start_location, plan)
    client.send_verify_request(start_location, plan)    
    response = client.get_response()
    print('Response received: ' + str(response))

    [start_location, plan] = create_sample_feasible_plan_2()
    print('Plan to verify:')
    print_plan(start_location, plan)
    client.send_verify_request(start_location, plan)    
    response = client.get_response()
    print('Response received: ' + str(response))

    client.create_update_node_client()
    attr = ['fridge', 'white', 'metal', 'heavy', 'big']
    client.send_update_node_request('fridge_1', 'object', attr, ['open', 'close'], 'open')
    response = client.get_response()
    print('Response received: ' + str(response))
    
    client.create_verify_client()
    print('Plan to verify:')
    print_plan(start_location, plan)
    client.send_verify_request(start_location, plan)    
    response = client.get_response()
    print('Response received: ' + str(response))

    client.create_dump_graph_client()
    format = 'yaml'
    client.send_dump_graph_request(format)
    response = client.get_response()
    # print('Response received: ' + str(response))
    if response.success:
        if format == 'json':
            graph = create_graph_from_json(response.kg_str)
        elif format == 'yaml':
            graph = create_graph_from_yaml(response.kg_str)
    plt.figure()
    pos = nx.spring_layout(graph)
    plt.title('Scene Graph')
    nx.draw(graph, pos, with_labels=True, font_weight='bold')
    edge_labels = nx.get_edge_attributes(graph, 'relationship')
    nx.draw_networkx_edge_labels(graph, pos, edge_labels=edge_labels)
    plt.ion()
    plt.show()
    input("Press Enter to close...")

    plt.close('all')


    
    
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
