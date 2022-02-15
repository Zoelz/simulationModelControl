from flask import Flask
import requests
from multiprocessing import Process, Queue, Value
import time
import rclpy
from nav_msgs.msg import Odometry
import dtweb


TWINBASE_IP = "https://dtid.org/"
HOST = "0.0.0.0"
PORT = 9050


class Peer:

    class Warehouse:

        def __init__(self, queue):
            self.dtid = "bcf39239-6722-49aa-9ec4-793f115eaed2"
            self.has_item = False


        def get_twin(self, DTID):
            twin_info = dtweb.client.fetch_dt_doc(TWINBASE_IP + DTID)
            return twin_info


        def driveToMyLocation(self, twin):
            twin_ip = self.get_twin(twin)["ip"]
            requests.get(twin_ip + "/driveTo/" + self.dtid)



    def __init__(self):
        self.queue = Queue()
        self.warehouse = self.Warehouse(self.queue)
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


        @app.route("/giveItem", methods=["GET"])
        def giveItem():
            if(self.warehouse.has_item):
                return str(False), 200
            self.warehouse.has_item = True
            return str(True), 200


        @app.route("/takeItem", methods=["GET"])
        def takeItem():
            if(self.warehouse.has_item):
                self.warehouse.has_item = False
                return str(True), 200
            return str(False), 200

        @app.route("/check_item", methods=["GET"])
        def check_item():
            return str(self.warehouse.has_item), 200


        return app

    def run_app(self):
        self.app.run(host=HOST, port=PORT)

if __name__ == "__main__":
    rclpy.init()
    p = Peer()
    while True:
        if not p.queue.empty():
            print(p.queue.get())
