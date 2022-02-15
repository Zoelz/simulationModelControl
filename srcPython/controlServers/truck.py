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
PORT = 7002

class Peer:

    class Truck:

        def getPosition(self):
            client = Client("opc.tcp://0.0.0.0:4842")
            client.connect()
            while True:
                self.x.value = client.get_node("ns=7;s=PositionX").get_value()
                self.y.value = client.get_node("ns=7;s=PositionY").get_value()




        def __init__(self, queue):
            self.dtid = "dde9d093-05bd-4512-8a23-1241e8809612"
            self.length = 0
            self.width = 0
            self.height = 0
            self.x = Value("f", 0.0)
            self.y = Value("f", 0.0)
            self.processUpdateLoc = Process(target=self.getPosition)
            self.processUpdateLoc.start()

            self.carrying_object = Value("b", True)
            self.moving = Value("b", False)
            self.drive_process = None
            self.queueDrive = Queue()
            self.processDrive = Process(target=self.drive, args=(self.queueDrive, ))
            self.processDrive.start()

            #self.start_ai_process = Process(target=self.ai)
            #self.start_ai_process.start()


        def drive(self, queue):
            client = Client("opc.tcp://0.0.0.0:4842")
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
                        vel = min(abs(target_ - self.x.value), 1.0)
                        if target_ < self.x.value:
                            dir = "f"
                        if target_ > self.x.value:
                            dir = "b"

                elif(isinstance(target, float)):
                    vel = min(abs(target - self.x.value), 1.0)
                    if target < self.x.value:
                        dir = "f"
                    if target > self.x.value:
                        dir = "b"

                else:
                    if dir == "f" and target < self.x.value and abs(target - self.x.value) < 0.01:
                        dir = None
                    if dir == "b" and target > self.x.value  and abs(target - self.x.value) < 0.01:
                        dir = None

                if dir == None:
                    client.get_node("ns=7;s=Forward").set_value(ua.Variant(False, ua.VariantType.Boolean))
                    client.get_node("ns=7;s=Backward").set_value(ua.Variant(False, ua.VariantType.Boolean))
                    client.get_node("ns=7;s=Speed").set_value(ua.Variant(0.0, ua.VariantType.Float))
                if dir == "f":
                    client.get_node("ns=7;s=Forward").set_value(ua.Variant(True, ua.VariantType.Boolean))
                    client.get_node("ns=7;s=Backward").set_value(ua.Variant(False, ua.VariantType.Boolean))
                    client.get_node("ns=7;s=Speed").set_value(ua.Variant(vel, ua.VariantType.Float))
                if dir == "b":
                    client.get_node("ns=7;s=Forward").set_value(ua.Variant(False, ua.VariantType.Boolean))
                    client.get_node("ns=7;s=Backward").set_value(ua.Variant(True, ua.VariantType.Boolean))
                    client.get_node("ns=7;s=Speed").set_value(ua.Variant(vel, ua.VariantType.Float))


        def get_twin(self, DTID):
            twin_info = dtweb.client.fetch_dt_doc(TWINBASE_IP + DTID)
            return twin_info


        def get_location(self, twin):
            twin_info = self.get_twin(twin)
            if(twin_info["x"] == None or twin_info["y"] == None):
                twin_ip = twin_info["ip"]
                loc = requests.get(twin_ip + "/get_location")
                loc = loc.json()
            else:
                loc = {"x": twin_info["x"], "y": twin_info["y"]}
            return loc


        def driveToMyLocation(self, twin):
            twin_ip = self.get_twin(twin)["ip"]
            requests.get(twin_ip + "/driveTo/" + self.dtid)


        def ai(self):
            while True:
                if(self.carrying_object.value == True):
                    listOfMachines = self.get_twin(self.get_twin(self.dtid)["parent"][0])["children"]


                    for i in range(len(listOfMachines)):
                        twin = self.get_twin(listOfMachines[i])
                        if(twin["type"] == "Warehouse"):
                            ip = twin["ip"]
                            warehouse_dtid = listOfMachines[i]
                            break

                    done = False

                    while not done:

                        for i in range(len(listOfMachines)):
                            twin = self.get_twin(listOfMachines[i])
                            if(twin["type"] == "Transport 3D"):
                                ip = twin["ip"]
                                if(requests.get(str(ip) + "/carrying_object").text == "0"):
                                    done = True
                                    transport_dtid = listOfMachines[i]
                                    break


                    requests.get(str(ip) + "/takeFrom/" + self.dtid)

                    requests.get(str(ip) + "/giveTo/" + warehouse_dtid)



    def __init__(self):
        self.queue = Queue()
        self.truck = self.Truck(self.queue)
        self.app = self.init_app()
        server = Process(target=self.run_app)
        server.start()


    def get_pahts(self, twin):
        twin_info = self.truck.get_twin(twin)
        pahts = []
        for path in twin_info["pahts"]:
            pahts.append(twin_info["ip"] + url)
        return pahts


    def create_request(self, twin_ip, path):
        req = requests.get(twin_ip + path)


    def init_app(self):
        app = Flask(__name__)

        @app.route("/")
        def index():
            return "index", 200


        @app.route("/get_location", methods=["GET"])
        def get_location():
            return {"x": self.truck.x.value, "y": self.truck.y.value}



        @app.route("/drive", methods=["GET", "POST"])
        def drive():
            if request.method == "GET":
                return render_template("postTargetCar.html"), 200
            else:
                if request.form.get("carBool") != None:
                    target = float(request.form.get("carTarget"))
                    self.truck.queueDrive.put(target)
                else:
                    self.truck.queueDrive.put("stop")
                return "Started driving", 200


        @app.route("/driveTo/<twin>", methods=["GET", "POST"])
        def driveToCrane(twin):
            if(";" in twin):
                targets = twin.split(";")
                self.truck.queueDrive.put(float(targets[0]))
                return "Started driving", 200
            self.truck.queueDrive.put(twin)
            return "Started driving", 200


        @app.route("/giveItem", methods=["GET"])
        def giveItem():
            if(self.truck.carrying_object.value):
                return str(False), 200
            self.truck.carrying_object.value = True
            return str(True), 200


        @app.route("/takeItem", methods=["GET"])
        def takeItem():
            if(self.truck.carrying_object.value):
                self.truck.carrying_object.value = False
                return str(True), 200
            return str(False), 200


        @app.route("/check_item", methods=["GET"])
        def check_item():
            return str(self.truck.carrying_object.value), 200


        return app

    def run_app(self):
        self.app.run(host=HOST, port=PORT)

if __name__ == "__main__":
    p = Peer()
    while True:
        if not p.queue.empty():
            print(p.queue.get())
