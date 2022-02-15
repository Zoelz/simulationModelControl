import time
from opcua import Server


if __name__ == "__main__":

    def add_variable(object, node, node_name, node_value):
        n = object.add_variable(node, node_name, node_value)
        n.set_writable()


    server = Server()
    server.set_endpoint("opc.tcp://0.0.0.0:4842")


    objs = server.get_objects_node()
    obj = objs.add_object("ns=7;s=SCF", "car")
    add_variable(obj, "ns=7;s=Forward", "Forward", bool(False))
    add_variable(obj, "ns=7;s=Backward", "Backward", bool(False))
    add_variable(obj, "ns=7;s=Speed", "Speed", float(0.0))
    add_variable(obj, "ns=7;s=AngularVelocity", "AngularVelocity", float(0.0))
    add_variable(obj, "ns=7;s=CarryingItem", "CarryingItem", bool(False))
    add_variable(obj, "ns=7;s=PositionX", "PositionX", float(0.0))
    add_variable(obj, "ns=7;s=PositionY", "PositionY", float(0.0))


    server.start()
