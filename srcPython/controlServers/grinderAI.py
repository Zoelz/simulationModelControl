from flask import Flask
import requests
from multiprocessing import Process, Queue, Value
import time
import rclpy
from nav_msgs.msg import Odometry
import dtweb


TWINBASE_IP = "https://dtid.org/"
HOST = "0.0.0.0"
PORT = 9000


class Peer:

    class Grinder:

        def __init__(self, queue):
            self.dtid = "3f31bdc2-1398-497f-be7d-3386a79523a6"
            self.queue = queue
            self.grinding = Value("b", False)
            self.start_grinding_process = None

            self.has_item = Value("b", False)

            self.start_ai_process = Process(target=self.ai)
            self.start_ai_process.start()

        def start_grinding(self):
            self.start_grinding_process = Process(target=self.start_grinding_p)
            self.start_grinding_process.start()


        def start_grinding_p(self):
            self.grinding.value = True
            time.sleep(60)
            self.grinding.value = False
            self.queue.put("Finished grinding")


        def get_twin(self, DTID):
            twin_info = dtweb.client.fetch_dt_doc(TWINBASE_IP + DTID)
            return twin_info


        def driveToMyLocation(self, twin):
            twin_ip = self.get_twin(twin)["ip"]
            requests.get(twin_ip + "/driveTo/" + self.dtid)


        def ai(self):
            while True:
                if(self.has_item.value == False):
                    listOfMachines = self.get_twin(self.get_twin(self.dtid)["parent"][0])["children"]

                    done = False

                    while not done:

                        for i in range(len(listOfMachines)):
                            twin = self.get_twin(listOfMachines[i])
                            if(twin["type"] == "Warehouse"):
                                ip = twin["ip"]
                                if(requests.get(str(ip) + "/check_item").text == "True"):
                                    done = True
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


                    requests.get(str(ip) + "/takeFrom/" + warehouse_dtid)

                    requests.get(str(ip) + "/giveTo/" + self.dtid)


                    time.sleep(5)


                elif not self.grinding.value:
                    print("?")

                    for i in range(len(listOfMachines)):
                            twin = get_twin(listOfMachines[i])
                            if(twin["type"] == "Warehouse"):
                                ip = twin["ip"]
                                warehouse_dtid = listOfMachines[i]
                                break
                    print("?")
                    while not done:

                        for i in range(len(listOfMachines)):
                            twin = get_twin(listOfMachines[i])
                            if(twin["type"] == "Transport 3D"):
                                ip = twin["ip"]
                                if(requests.get(str(ip) + "/carrying_object").text == "0"):
                                    done = True
                                    transport_dtid = listOfMachines[i]
                                    break
                    print("?")
                    requests.get(str(ip) + "/takeFrom/" + self.dtid)

                    requests.get(str(ip) + "/giveTo/" + warehouse_dtid)


    def __init__(self):
        self.queue = Queue()
        self.grinder = self.Grinder(self.queue)
        self.app = self.init_app()
        server = Process(target=self.run_app)
        server.start()


    def get_location(self, twin):
        twin_ip = self.grinder.get_twin(twin)["ip"]
        loc = requests.get(twin_ip + "/get_location/")
        print(loc[x], loc[y], loc[z])


    def init_app(self):
        app = Flask(__name__)

        @app.route("/")
        def index():
            return "index", 200


        @app.route("/get_location", methods=["GET"])
        def get_location():
            x = self.grinder.x.value
            y = self.grinder.y.value
            z = self.grinder.z.value
            return {"x": x, "y": y, "z": z}, 200


        @app.route("/grinding", methods=["GET"])
        def active():
            return {"Grinding": self.grinder.grinding.value}, 200


        @app.route("/start_grinding", methods=["GET"])
        def start_grinding():
            self.grinder.start_grinding()
            return "Started grinding", 200


        @app.route("/giveItem", methods=["GET"])
        def giveItem():
            if(self.grinder.has_item.value):
                return str(False), 200
            self.grinder.has_item = Value("b", True)
            return str(True), 200


        @app.route("/takeItem", methods=["GET"])
        def takeItem():
            if(self.grinder.has_item.value):
                self.grinder.has_item = Value("b", False)
                return str(True), 200
            return str(False), 200

        @app.route("/check_item", methods=["GET"])
        def check_item():
            return str(self.grinder.has_item.value), 200


        return app

    def run_app(self):
        self.app.run(host=HOST, port=PORT)

if __name__ == "__main__":
    rclpy.init()
    p = Peer()
    while True:
        if not p.queue.empty():
            print(p.queue.get())
