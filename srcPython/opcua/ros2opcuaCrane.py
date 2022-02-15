from opcua import Client
from opcua import ua
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy


if __name__ == "__main__":
    rclpy.init()
    client = Client("opc.tcp://0.0.0.0:4840")
    client.connect()

    def writeBridgeLocation(msg):
        client.get_node("ns=7;s=SCF.PLC.DX_Custom_V.Status.Bridge.Position.Position_m").set_value(ua.Variant(msg.pose.pose.position.x, ua.VariantType.Float))

    def writeTrolleyLocation(msg):
        client.get_node("ns=7;s=SCF.PLC.DX_Custom_V.Status.Trolley.Position.Position_m").set_value(ua.Variant(msg.pose.pose.position.y, ua.VariantType.Float))

    def writeHoistLocation(msg):
        client.get_node("ns=7;s=SCF.PLC.DX_Custom_V.Status.Hoist.Position.Position_m").set_value(ua.Variant(msg.pose.pose.position.z, ua.VariantType.Float))


    def writeBridgeDrive(publisher):
        msg = Twist()
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.linear.x = 0.0
        f = client.get_node("ns=7;s=SCF.PLC.DX_Custom_V.Controls.Bridge.Forward").get_value()
        b = client.get_node("ns=7;s=SCF.PLC.DX_Custom_V.Controls.Bridge.Backward").get_value()
        s = client.get_node("ns=7;s=SCF.PLC.DX_Custom_V.Controls.Bridge.Speed").get_value()
        if(f == True):
            msg.linear.x = s
        if(b == True):
            msg.linear.x = -s
        publisher.publish(msg)

    def writeTrolleyDrive(publisher):
        msg = Twist()
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.linear.x = 0.0
        f = client.get_node("ns=7;s=SCF.PLC.DX_Custom_V.Controls.Trolley.Forward").get_value()
        b = client.get_node("ns=7;s=SCF.PLC.DX_Custom_V.Controls.Trolley.Backward").get_value()
        s = client.get_node("ns=7;s=SCF.PLC.DX_Custom_V.Controls.Trolley.Speed").get_value()
        if(f == True):
            msg.linear.x = s
        if(b == True):
            msg.linear.x = -s
        publisher.publish(msg)

    def writeHoistDrive(publisher):
        msg = Twist()
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.linear.x = 0.0
        f = client.get_node("ns=7;s=SCF.PLC.DX_Custom_V.Controls.Hoist.Forward").get_value()
        b = client.get_node("ns=7;s=SCF.PLC.DX_Custom_V.Controls.Hoist.Backward").get_value()
        s = client.get_node("ns=7;s=SCF.PLC.DX_Custom_V.Controls.Hoist.Speed").get_value()
        if(f == True):
            msg.linear.x = s
        if(b == True):
            msg.linear.x = -s
        publisher.publish(msg)


    nodeCartPub = rclpy.create_node("nodeCartPub")
    publisherCart = nodeCartPub.create_publisher(Twist, "/cart/cmd_vel", 10)

    nodeBridgePub = rclpy.create_node("nodeBridgePub")
    publisherBridge = nodeBridgePub.create_publisher(Twist, "/bridge/cmd_vel", 10)

    nodeSubBridge = rclpy.create_node("bridgeNodeSub")
    nodeSubTrolley = rclpy.create_node("cartNodeSub")
    subscriberBridge = nodeSubBridge.create_subscription(Odometry, "/bridge/odom", writeBridgeLocation, 10)
    subscriberTrolley = nodeSubTrolley.create_subscription(Odometry, "/cart/odom", writeTrolleyLocation, 10)

    while True:
        rclpy.spin_once(nodeSubBridge)
        rclpy.spin_once(nodeSubTrolley)
        writeBridgeDrive(publisherBridge)
        writeTrolleyDrive(publisherCart)
