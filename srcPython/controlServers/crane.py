from flask import Flask, render_template, request
import requests
from multiprocessing import Process, Queue, Value
import time
import math
from opcua import Client
from opcua import ua
import dtweb

TWINBASE_IP = "https://dtid.org/"
HOST = "0.0.0.0"
PORT = 7000

class Peer:

    class Crane:

        def getPosition(self, object, objectName):
            client = Client("opc.tcp://0.0.0.0:4840")
            client.connect()
            if objectName == "bridge":
                while True:
                    object.x.value = client.get_node("ns=7;s=SCF.PLC.DX_Custom_V.Status.Bridge.Position.Position_m").get_value()

            if objectName == "cart":
                while True:
                    object.y.value = client.get_node("ns=7;s=SCF.PLC.DX_Custom_V.Status.Trolley.Position.Position_m").get_value()


        class Bridge:

            def __init__(self, getPositionFunc):
                self.length = 0
                self.width = 0
                self.height = 0
                self.x = Value("f", 0.0)
                self.processUpdateLoc = Process(target=getPositionFunc, args=(self, "bridge"))
                self.processUpdateLoc.start()


        class Cart:

            def __init__(self, getPositionFunc):
                self.length = 0
                self.width = 0
                self.height = 0
                self.y = Value("f", 0.0)
                self.processUpdateLoc = Process(target=getPositionFunc, args=(self, "cart"))
                self.processUpdateLoc.start()


        def __init__(self, queue):
            self.dtid = "a346d686-fa08-4eab-86b7-1e5367d46e98"
            self.bridge = self.Bridge(self.getPosition)
            self.cart = self.Cart(self.getPosition)
            self.carrying_object = Value("b", False)
            self.moving_horizontal = Value("b", False)
            self.moving_vertical = Value("b", False)
            self.moving_z = Value("b", False)
            self.drive_horizontal_process = None
            self.drive_vertical_process = None
            self.drive_z_process = None
            self.queueHorizontal = Queue()
            self.processDriveHorizontal = Process(target=self.drive_horizontal, args=(self.queueHorizontal, ))
            self.processDriveHorizontal.start()

            self.queueVertical = Queue()
            self.processDriveVertical = Process(target=self.drive_vertical, args=(self.queueVertical, ))
            self.processDriveVertical.start()

            self.otherMachines = {} # {"dtid: {ip: ip, last_time_updated: "hh::mm::ss"}"}



        def drive_horizontal(self, queue):
            client = Client("opc.tcp://0.0.0.0:4840")
            client.connect()
            target = None
            dir = None
            while True:
                if not queue.empty():
                    target = queue.get()


                if(isinstance(target, str)):
                    if target == "stop":
                        dir = None
                    else:
                        target_ = float(self.get_location(target)["x"])
                        vel = min(abs(target_ - self.bridge.x.value), 1.0)
                        if target_ < self.bridge.x.value:
                            dir = "f"
                        if target_ > self.bridge.x.value:
                            dir = "b"


                elif(isinstance(target, float)):
                    vel = min(abs(target - self.bridge.x.value), 1.0)
                    if target < self.bridge.x.value:
                        dir = "f"
                    if target > self.bridge.x.value:
                        dir = "b"

                else:
                    if dir == "f" and target < self.bridge.x.value and abs(target - self.bridge.x.value) < 0.01:
                        dir = None
                    if dir == "b" and target > self.bridge.x.value  and abs(target - self.bridge.x.value) < 0.01:
                        dir = None

                if dir == None:
                    client.get_node("ns=7;s=SCF.PLC.DX_Custom_V.Controls.Bridge.Forward").set_value(ua.Variant(False, ua.VariantType.Boolean))
                    client.get_node("ns=7;s=SCF.PLC.DX_Custom_V.Controls.Bridge.Backward").set_value(ua.Variant(False, ua.VariantType.Boolean))
                    client.get_node("ns=7;s=SCF.PLC.DX_Custom_V.Controls.Bridge.Speed").set_value(ua.Variant(0.0, ua.VariantType.Float))
                if dir == "f":
                    client.get_node("ns=7;s=SCF.PLC.DX_Custom_V.Controls.Bridge.Forward").set_value(ua.Variant(True, ua.VariantType.Boolean))
                    client.get_node("ns=7;s=SCF.PLC.DX_Custom_V.Controls.Bridge.Backward").set_value(ua.Variant(False, ua.VariantType.Boolean))
                    client.get_node("ns=7;s=SCF.PLC.DX_Custom_V.Controls.Bridge.Speed").set_value(ua.Variant(vel, ua.VariantType.Float))
                if dir == "b":
                    client.get_node("ns=7;s=SCF.PLC.DX_Custom_V.Controls.Bridge.Forward").set_value(ua.Variant(False, ua.VariantType.Boolean))
                    client.get_node("ns=7;s=SCF.PLC.DX_Custom_V.Controls.Bridge.Backward").set_value(ua.Variant(True, ua.VariantType.Boolean))
                    client.get_node("ns=7;s=SCF.PLC.DX_Custom_V.Controls.Bridge.Speed").set_value(ua.Variant(vel, ua.VariantType.Float))


        def drive_vertical(self, queue):
            client = Client("opc.tcp://0.0.0.0:4840")
            client.connect()
            target = None
            dir = None
            while True:
                if not queue.empty():
                    target = queue.get()


                if(isinstance(target, str)):
                    if target == "stop":
                        dir = None
                    else:
                        target_ = float(self.get_location(target)["y"])
                        vel = min(abs(target_ - self.cart.y.value), 1.0)
                        if target_ < self.cart.y.value:
                            dir = "f"
                        if target_ > self.cart.y.value:
                            dir = "b"

                elif(isinstance(target, float)):
                    vel = min(abs(target - self.cart.y.value), 1.0)
                    if target < self.cart.y.value:
                        dir = "f"
                    if target > self.cart.y.value:
                        dir = "b"

                else:
                    if dir == "f" and target < self.cart.y.value and abs(target - self.cart.y.value) < 0.01:
                        dir = None
                    if dir == "b" and target > self.cart.y.value  and abs(target - self.cart.y.value) < 0.01:
                        dir = None

                if dir == None:
                    client.get_node("ns=7;s=SCF.PLC.DX_Custom_V.Controls.Trolley.Forward").set_value(ua.Variant(False, ua.VariantType.Boolean))
                    client.get_node("ns=7;s=SCF.PLC.DX_Custom_V.Controls.Trolley.Backward").set_value(ua.Variant(False, ua.VariantType.Boolean))
                    client.get_node("ns=7;s=SCF.PLC.DX_Custom_V.Controls.Trolley.Speed").set_value(ua.Variant(0.0, ua.VariantType.Float))
                if dir == "f":
                    client.get_node("ns=7;s=SCF.PLC.DX_Custom_V.Controls.Trolley.Forward").set_value(ua.Variant(True, ua.VariantType.Boolean))
                    client.get_node("ns=7;s=SCF.PLC.DX_Custom_V.Controls.Trolley.Backward").set_value(ua.Variant(False, ua.VariantType.Boolean))
                    client.get_node("ns=7;s=SCF.PLC.DX_Custom_V.Controls.Trolley.Speed").set_value(ua.Variant(vel, ua.VariantType.Float))
                if dir == "b":
                    client.get_node("ns=7;s=SCF.PLC.DX_Custom_V.Controls.Trolley.Forward").set_value(ua.Variant(False, ua.VariantType.Boolean))
                    client.get_node("ns=7;s=SCF.PLC.DX_Custom_V.Controls.Trolley.Backward").set_value(ua.Variant(True, ua.VariantType.Boolean))
                    client.get_node("ns=7;s=SCF.PLC.DX_Custom_V.Controls.Trolley.Speed").set_value(ua.Variant(vel, ua.VariantType.Float))


        def driveToMyLocation(self, twin):
            twin_ip = self.get_twin(twin)["ip"]
            requests.get(twin_ip + "/driveTo/" + self.dtid)


        def get_twin(self, DTID):
            twin_info = dtweb.client.fetch_dt_doc(TWINBASE_IP + DTID)
            return twin_info


        def get_location(self, twin):
            twin_info = self.get_twin(twin)
            if(twin_info["location"]["x"] == "None" or twin_info["location"]["y"] == "None"):
                twin_ip = twin_info["ip"]
                loc = requests.get(twin_ip + "/get_location")
                loc = loc.json()
            else:
                loc = {"x": twin_info["location"]["x"], "y": twin_info["location"]["y"]}
            return loc



    def __init__(self):
        self.queue = Queue()
        self.crane = self.Crane(self.queue)
        self.app = self.init_app()
        server = Process(target=self.run_app)
        server.start()


    def get_pahts(self, twin):
        twin_info = self.crane.get_twin(twin)
        pahts = []
        for path in twin_info["pahts"]:
            pahts.append(twin_info["ip"] + url)
        return pahts


    def create_request(self, twin_ip, path):
        req = requests.get(twin_ip + path)


    def check_grinder_active(self):
        twin_ip = self.crane.get_twin("grinder")["ip"]
        active = requests.get(twin_ip + "/grinding")
        return active.json()


    def init_app(self):
        app = Flask(__name__)

        @app.route("/")
        def index():
            return "index", 200


        @app.route("/get_location", methods=["GET"])
        def get_location():
            return {"x": self.crane.bridge.x.value, "y": self.crane.cart.y.value}, 200



        @app.route("/drive", methods=["GET", "POST"])
        def drive():
            if request.method == "GET":
                return render_template("postTarget.html"), 200
            else:
                if request.form.get("bridgeBool") != None:
                    target = float(request.form.get("bridgeTarget"))
                    self.crane.queueHorizontal.put(target)
                else:
                    self.crane.queueHorizontal.put("stop")
                if request.form.get("cartBool") != None:
                    target = float(request.form.get("cartTarget"))
                    self.crane.queueVertical.put(target)
                else:
                    self.crane.queueVertical.put("stop")
                return "Started driving", 200



        @app.route("/driveTo/<twin>", methods=["GET", "POST"])
        def driveToCrane(twin):
            if(";" in twin):
                targets = twin.split(";")
                self.crane.queueVertical.put(float(targets[0]))
                self.crane.queueHorizontal.put(float(targets[1]))
                return "Started driving", 200
            self.crane.queueHorizontal.put(twin)
            self.crane.queueVertical.put(twin)
            return "Started driving", 200


        @app.route("/takeFrom/<twin>", methods=["GET", "POST"])
        def takeFrom(twin):

            if not self.crane.carrying_object.value:
                if(";" in twin):
                    targets = twin.split(";")
                    self.crane.queueVertical.put(float(targets[1]))
                    self.crane.queueHorizontal.put(float(targets[0]))

                    targetY = float(targets[1])
                    targetX = float(targets[0])

                    while abs(targetY - self.crane.cart.y.value) > 0.2 or abs(targetX - self.crane.bridge.x.value) > 0.2:
                        targetY = float(targets[1])
                        targetX = float(targets[0])

                    self.crane.carrying_object = Value("b", True)
                    requests.get("http://localhost:" + targets[2] + "/takeItem")

                    return "Started driving", 200

                self.crane.queueHorizontal.put(twin)
                self.crane.queueVertical.put(twin)

                targetY = float(self.crane.get_location(twin)["y"])
                targetX = float(self.crane.get_location(twin)["x"])

                while abs(targetY - self.crane.cart.y.value) > 0.2 or abs(targetX - self.crane.bridge.x.value) > 0.2:
                    targetY = float(self.crane.get_location(twin)["y"])
                    targetX = float(self.crane.get_location(twin)["x"])

                self.crane.carrying_object = Value("b", True)

                twin_ip = self.crane.get_twin(twin)["ip"]
                requests.get(twin_ip + "/takeItem")

                return "Started driving", 200

            return "Already carrying object", 200


        @app.route("/giveTo/<twin>", methods=["GET", "POST"])
        def giveTo(twin):


            if self.crane.carrying_object.value:

                if(";" in twin):
                    targets = twin.split(";")
                    self.crane.queueVertical.put(float(targets[1]))
                    self.crane.queueHorizontal.put(float(targets[0]))

                    targetY = float(targets[1])
                    targetX = float(targets[0])

                    while abs(targetY - self.crane.cart.y.value) > 0.2 or abs(targetX - self.crane.bridge.x.value) > 0.2:
                        targetY = float(targets[1])
                        targetX = float(targets[0])

                    self.crane.carrying_object = Value("b", True)
                    requests.get("http://localhost:" + targets[2] + "/giveItem")

                    return "Started driving", 200

                self.crane.queueHorizontal.put(twin)
                self.crane.queueVertical.put(twin)

                targetY = float(self.crane.get_location(twin)["y"])
                targetX = float(self.crane.get_location(twin)["x"])

                while abs(targetY - self.crane.cart.y.value) > 0.2 or abs(targetX - self.crane.bridge.x.value) > 0.2:
                    targetY = float(self.crane.get_location(twin)["y"])
                    targetX = float(self.crane.get_location(twin)["x"])

                self.crane.carrying_object = Value("b", False)

                twin_ip = self.crane.get_twin(twin)["ip"]
                requests.get(twin_ip + "/giveItem")


                return "Started driving", 200

            return "Not yet carrying object", 200


        @app.route("/carrying_object", methods=["GET", "POST"])
        def carrying_object():

            return str(self.crane.carrying_object.value), 200


        @app.route("/fromTo/<tdid1>/<tdid2>", methods=["GET", "POST"])
        def fromTo(tdid1, tdid2):

            if(";" in tdid1):
                targets = tdid1.split(";")
                self.crane.queueVertical.put(float(targets[1]))
                self.crane.queueHorizontal.put(float(targets[0]))

                targetY = float(targets[1])
                targetX = float(targets[0])

                while abs(targetY - self.crane.cart.y.value) > 0.2 or abs(targetX - self.crane.bridge.x.value) > 0.2:
                    targetY = float(targets[1])
                    targetX = float(targets[0])

                targets = tdid2.split(";")

                self.crane.queueHorizontal.put(float(targets[0]))
                self.crane.queueVertical.put(float(targets[1]))

                return "Started driving", 200

            self.crane.queueHorizontal.put(tdid1)
            self.crane.queueVertical.put(tdid1)

            targetY = float(self.crane.get_location(tdid1)["y"])
            targetX = float(self.crane.get_location(tdid1)["x"])

            while abs(targetY - self.crane.cart.y.value) > 0.2 or abs(targetX - self.crane.bridge.x.value) > 0.2:
                targetY = float(self.crane.get_location(tdid1)["y"])
                targetX = float(self.crane.get_location(tdid1)["x"])

            self.crane.queueHorizontal.put(tdid2)
            self.crane.queueVertical.put(tdid2)

            return "Started driving", 200

        return app

    def run_app(self):
        self.app.run(host=HOST, port=PORT)

if __name__ == "__main__":
    p = Peer()
    while True:
        if not p.queue.empty():
            print(p.queue.get())
