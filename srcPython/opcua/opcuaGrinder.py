import time
from opcua import Server


if __name__ == "__main__":

    def add_variable(object, node, node_name, node_value):
        n = object.add_variable(node, node_name, node_value)
        n.set_writable()


    server = Server()
    server.set_endpoint("opc.tcp://0.0.0.0:4841")


    objs = server.get_objects_node()
    obj = objs.add_object("ns=7;s=SCF", "Grinder")
    add_variable(obj, "ns=7;s=startGrinding", "Grinding", bool(False))


    server.start()
