import time
from opcua import Server


if __name__ == "__main__":

    def add_variable(object, node, node_name, node_value):
        n = object.add_variable(node, node_name, node_value)
        n.set_writable()


    server = Server()
    server.set_endpoint("opc.tcp://0.0.0.0:4840")


    objs = server.get_objects_node()
    obj = objs.add_object("ns=7;s=SCF", "Crane")
    add_variable(obj, "ns=7;s=SCF.PLC.DX_Custom_V.Controls.Watchdog", "Watchdog", int(0))
    add_variable(obj, "ns=7;s=SCF.PLC.DX_Custom_V.Controls.AccessCode", "AccessCode", int(0))
    add_variable(obj, "ns=7;s=SCF.PLC.DX_Custom_V.Controls.Hoist.Up", "Hoist.Up", bool(False))
    add_variable(obj, "ns=7;s=SCF.PLC.DX_Custom_V.Controls.Hoist.Down", "Hoist.Down", bool(False))
    add_variable(obj, "ns=7;s=SCF.PLC.DX_Custom_V.Controls.Hoist.Speed", "Hoist.Speed", float(0.0))
    add_variable(obj, "ns=7;s=SCF.PLC.DX_Custom_V.Controls.Trolley.Forward", "Trolley.Forward", bool(False))
    add_variable(obj, "ns=7;s=SCF.PLC.DX_Custom_V.Controls.Trolley.Backward", "Trolley.Backward", bool(False))
    add_variable(obj, "ns=7;s=SCF.PLC.DX_Custom_V.Controls.Trolley.Speed", "Trolley.Speed", float(0.0))
    add_variable(obj, "ns=7;s=SCF.PLC.DX_Custom_V.Controls.Bridge.Forward", "Bridge.Forward", bool(False))
    add_variable(obj, "ns=7;s=SCF.PLC.DX_Custom_V.Controls.Bridge.Backward", "Bridge.Backward", bool(False))
    add_variable(obj, "ns=7;s=SCF.PLC.DX_Custom_V.Controls.Bridge.Speed", "Bridge.Speed", float(0.0))

    #obj.add_variable("ns=7;s=SCF.PLC.DX_Custom_V.Controls.TargetPositioning.SelectionInUse", "SelectionInUse", bool(False))
    #obj.add_variable("ns=7;s=SCF.PLC.DX_Custom_V.Controls.TargetPositioning.DriveToTarget", "DriveToTarget", bool(False))
    #obj.add_variable("ns=7;s=SCF.PLC.DX_Custom_V.Controls.TargetPositioning.DriveToHome", "DriveToHome", bool(False))
    #obj.add_variable("ns=7;s=SCF.PLC.DX_Custom_V.Controls.TargetPositioning.Target", "Target", int(0))
    #obj.add_variable("ns=7;s=SCF.PLC.DX_Custom_V.Controls.TargetPositioning.Home", "Home", int(0))

    #obj.add_variable("ns=7;s=SCF.PLC.DX_Custom_V.Status.ReadingControls", "ReadingControls", bool(False))
    #obj.add_variable("ns=7;s=SCF.PLC.DX_Custom_V.Status.WatchDogFault", "WatchDogFault", bool(False))

    #obj.add_variable("ns=7;s=SCF.PLC.DX_Custom_V.RadioSelection.Inching", "Inching", bool(False))
    #obj.add_variable("ns=7;s=SCF.PLC.DX_Custom_V.RadioSelection.MicroSpeed", "MicroSpeed", bool(False))
    #obj.add_variable("ns=7;s=SCF.PLC.DX_Custom_V.RadioSelection.RopeAngleFeaturesBypass", "RopeAngleFeaturesBypass", bool(False))
    #obj.add_variable("ns=7;s=SCF.PLC.DX_Custom_V.RadioSelection.SwayControl", "SwayControl", bool(False))
    #obj.add_variable("ns=7;s=SCF.PLC.DX_Custom_V.RadioSelection.SwayControl_SlingLength_mm", "SwayControl_SlingLength_mm", int(0))


    obj.add_variable("ns=7;s=SCF.PLC.DX_Custom_V.Status.Hoist.Diagnostics.Ok", "Diagnostics.Ok", bool(False))
    #Lisää loput


    add_variable(obj, "ns=7;s=SCF.PLC.DX_Custom_V.Status.Hoist.Position.Position_m", "HoistPosition", float(0.0))
    add_variable(obj, "ns=7;s=SCF.PLC.DX_Custom_V.Status.Bridge.Position.Position_m", "BridgePosition", float(0.0))
    add_variable(obj, "ns=7;s=SCF.PLC.DX_Custom_V.Status.Trolley.Position.Position_m", "TrolleyPosition", float(0.0))


    server.start()
