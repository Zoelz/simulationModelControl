from flask import Flask, render_template, request
import requests
from multiprocessing import Process, Queue, Value
import time
import math
from opcua import Client
from opcua import ua
import dtweb


TWINBASE_IP = "https://dtid.org/"

def get_twin(DTID):
    twin_info = dtweb.client.fetch_dt_doc(TWINBASE_IP + DTID)
    return twin_info


if __name__ == "__main__":
    factory_dtid = "d1816959-8a88-40b1-9bfd-8a670b629083"
    owner_dtid = "dde9d093-05bd-4512-8a23-1241e8809612"

    listOfMachines = get_twin(factory_dtid)["children"]

    done = False

    while not done:

        for i in range(len(listOfMachines)):
            twin = get_twin(listOfMachines[i])
            if(twin["type"] == "Transport 3D"):
                ip = twin["ip"]
                if(requests.get(str(ip) + "/carrying_object").text == "0"):
                    done = True
                    future_owner = listOfMachines[i]
                    break

    twin2 = get_twin(owner_dtid)
    targetLocations = twin2["location"]
    if(targetLocations["x"] == "None"):
        targetLocations = requests.get(twin2["ip"] + "/get_location").json()
    if "ip" in twin2:
        locations = str(targetLocations["x"]) + ";" + str(targetLocations["y"]) + ";" + str(twin2["ip"].split(":")[-1])
    else:
        locations = str(targetLocations["x"]) + ";" + str(targetLocations["y"])

    requests.get(str(ip) + "/takeFrom/" + locations)
    owner_dtid = future_owner

    freeMachine = False

    for i in range(len(listOfMachines)):
        twin = get_twin(listOfMachines[i])
        if(twin["type"] == "Grinding machine"):
            test_ip = twin["ip"]
            if(requests.get(str(test_ip) + "/check_item").text == "0"):
                freeMachine = True
                future_owner = listOfMachines[i]
                break

    if not freeMachine:
        for i in range(len(listOfMachines)):
            twin = get_twin(listOfMachines[i])
            if(twin["type"] == "Warehouse"):
                ip = twin["ip"]
                target = listOfMachines[i]


        twin2 = get_twin(owner_dtid)
        targetLocations = twin2["location"]
        if(targetLocations["x"] == "None"):
            targetLocations = requests.get(twin2["ip"] + "/get_location").json()
        if "ip" in twin2:
            locations = str(targetLocations["x"]) + ";" + str(targetLocations["y"]) + ";" + str(twin2["ip"].split(":")[-1])
        else:
            locations = str(targetLocations["x"]) + ";" + str(targetLocations["y"])

        requests.get(str(get_twin(owner_dtid)["ip"]) + "/giveTo/" + locations)
        owner_dtid = target


        done = False

        while not done:
            for i in range(len(listOfMachines)):
                twin = get_twin(listOfMachines[i])
                if(twin["type"] == "Grinding machine"):
                    ip = twin["ip"]
                    if(requests.get(str(ip) + "/check_item").text == "0"):
                        future_owner = listOfMachines[i]
                        done = True
                        break



        done = False

        while not done:

            for i in range(len(listOfMachines)):
                twin = get_twin(listOfMachines[i])
                if(twin["type"] == "Transport 3D"):
                    ip = twin["ip"]
                    if(requests.get(str(ip) + "/carrying_object").text == "0"):
                        done = True
                        transporter_future_owner = listOfMachines[i]
                        break

        twin2 = get_twin(owner_dtid)
        targetLocations = twin2["location"]
        if(targetLocations["x"] == "None"):
            targetLocations = requests.get(twin2["ip"] + "/get_location").json()
        if "ip" in twin2:
            locations = str(targetLocations["x"]) + ";" + str(targetLocations["y"]) + ";" + str(twin2["ip"].split(":")[-1])
        else:
            locations = str(targetLocations["x"]) + ";" + str(targetLocations["y"])

        requests.get(str(ip) + "/takeFrom/" + locations)
        owner_dtid = transporter_future_owner


    twin2 = get_twin(future_owner)
    targetLocations = twin2["location"]
    if(targetLocations["x"] == "None"):
        targetLocations = requests.get(twin2["ip"] + "/get_location").json()
    if "ip" in twin2:
        locations = str(targetLocations["x"]) + ";" + str(targetLocations["y"]) + ";" + str(twin2["ip"].split(":")[-1])
    else:
        locations = str(targetLocations["x"]) + ";" + str(targetLocations["y"])

    requests.get(str(ip) + "/giveTo/" + locations)

    owner_dtid = future_owner
    owner = get_twin(owner_dtid)
    requests.get(str(owner["ip"]) + "/start_grinding")
