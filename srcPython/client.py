import requests
import dtweb

TWINBASE_IP = "https://dtid.org/"

def get_twin(DTID):
    twin_info = dtweb.client.fetch_dt_doc(TWINBASE_IP + DTID)
    return twin_info


if __name__ == "__main__":
    factory_dtid = "d1816959-8a88-40b1-9bfd-8a670b629083"

    listOfMachines = get_twin(factory_dtid)["children"]
    for i in range(len(listOfMachines)):
        twin = get_twin(listOfMachines[i])
        print(str(i) + ": name: " + twin["name"] + " : type: " + twin["type"])

    while True:

        machine = int(input("Machine to control: "))

        functions = get_twin(listOfMachines[machine])["functions"]

        for i in range(len(functions)):
            print(str(i) + ": " + functions[i])

        f = int(input("Function to use: "))

        count = functions[f].count("<")

        ip = get_twin(listOfMachines[machine])["ip"]

        if(count > 0):
            targets = listOfMachines[int(input("Target: "))]
            twin = get_twin(targets)
            targetLocations = twin["location"]

            if(targetLocations["x"] == "None"):
                targetLocations = requests.get(twin["ip"] + "/get_location").json()

            if "ip" in twin:
                locations = str(targetLocations["x"]) + ";" + str(targetLocations["y"]) + ";" + str(twin["ip"].split(":")[-1])
            else:
                locations = str(targetLocations["x"]) + ";" + str(targetLocations["y"])
            count = count - 1

            while(count > 0):
                targets = listOfMachines[int(input("Target: "))]
                twin = get_twin(targets)
                targetLocations = twin["location"]

                if(targetLocations["x"] == "None"):
                    targetLocations = requests.get(twin["ip"] + "/get_location").json()
                if "ip" in twin:
                    locations += "/" + str(targetLocations["x"]) + ";" + str(targetLocations["y"]) + ";" + str(twin["ip"].split(":")[-1])
                else:
                    locations += "/" + str(targetLocations["x"]) + ";" + str(targetLocations["y"])
                count = count - 1

            response = requests.get(str(ip) + functions[f].split("<")[0] + str(locations))

        else:
            response = requests.get(str(ip) + functions[f])

        print(response.text)
