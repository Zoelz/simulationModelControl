from opcua import Client
from opcua import ua
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy


if __name__ == "__main__":
    rclpy.init()
    client = Client("opc.tcp://0.0.0.0:4842")
    client.connect()

    def writePosition(msg):
        client.get_node("ns=7;s=PositionX").set_value(ua.Variant(msg.pose.pose.position.x, ua.VariantType.Float))
        client.get_node("ns=7;s=PositionY").set_value(ua.Variant(msg.pose.pose.position.y, ua.VariantType.Float))


    def writeBridgeDrive(publisher):
        msg = Twist()
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.linear.x = 0.0
        f = client.get_node("ns=7;s=Forward").get_value()
        b = client.get_node("ns=7;s=Backward").get_value()
        s = client.get_node("ns=7;s=Speed").get_value()
        a = client.get_node("ns=7;s=AngularVelocity").get_value()
        if(f == True):
            msg.linear.x = s
        if(b == True):
            msg.linear.x = -s
        msg.angular.z = a
        publisher.publish(msg)


    nodeCarPub = rclpy.create_node("nodeCarPub")
    publisherCar = nodeCarPub.create_publisher(Twist, "/car/cmd_vel", 10)


    nodeSubCar = rclpy.create_node("carNodeSub")
    subscriberCar = nodeSubCar.create_subscription(Odometry, "/car/odom", writePosition, 10)

    while True:
        rclpy.spin_once(nodeSubCar)
        writeBridgeDrive(publisherCar)
