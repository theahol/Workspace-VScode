import sys
import socket
sys.path.insert(0, "..")
import time
import threading
from threading import Thread, Lock
from moto.simple_message import *
from moto import motion_connection
from moto.simple_message_connection import SimpleMessageConnection
from moto.simple_message import SimpleMessage
from moto import Moto
from moto import Moto, ControlGroupDefinition
from moto.simple_message import (
    JointFeedbackEx,
    JointTrajPtExData,
    JointTrajPtFull,
    JointTrajPtFullEx,
    ValidFields,
    JointFeedback,
    MsgType, RobotStatus,
    SimpleMessage,
    MOT_MAX_GR,
)

import copy
from opcua import ua, Server
import numpy as np

#robotip:192.168.255.200  
#robotip:192.168.255.200  

#m er et object(?) i/av klassen Moto

robot: Moto = Moto(
        "localhost",
        [ControlGroupDefinition("robot", 0, 6, ["s", "l", "u", "r", "b", "t"])],
    )

'''
først: robot.motion.start_servos()
så: robot.motion.start_trajectory_mode()
så sjekk status:
status = robot.motion.check_motion_ready()
print("status of robot is: ", status)'''


robot.motion.start_servos()
robot.motion.start_trajectory_mode()

#print("##############\nPosition of joint {}\n##############".format(type(robot.state.joint_feedback(0))))

p0 = JointTrajPtFull(
            groupno=0,
            sequence=0,
            valid_fields=ValidFields.TIME | ValidFields.POSITION | ValidFields.VELOCITY,
            time=0,
            pos = robot.state.joint_feedback(0).pos,
            vel = [0.0]*10,
            acc=robot.state.joint_feedback(0).acc
        )
robot.motion.send_joint_trajectory_point(p0)

p1 = JointTrajPtFull(
            groupno=0,
            sequence=1,
            valid_fields=ValidFields.TIME | ValidFields.POSITION | ValidFields.VELOCITY,
            time=10.0,
            pos=np.deg2rad([40.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            vel=[0.0] * 10,
            acc=[0.0] * 10
        )

robot.motion.send_joint_trajectory_point(p1)
time.sleep(10)

print("##############\nPosition of joint {}\n##############".format(type(robot.state.joint_feedback(0))))
position = robot.state.joint_feedback(0).pos
print("current position of robot is: ", position)


def disconnect (self):
    client.close()

if __name__ == "__main__":

    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # setup our server
    server = Server()
    server.set_endpoint("opc.tcp://10.24.11.202:4840/motopcua/server/")

    # setup our own namespace, not really necessary but should as spec
    uri = "http://server.motopcua.github.io"
    idx = server.register_namespace(uri)

    # get Objects node, this is where we should put our nodes
    objects = server.get_objects_node()

    # populating our address space
    myobj = server.nodes.objects.add_object(idx, "Moto")
    myvar = myobj.add_variable(idx, "MyVariable", 6.7)

    myvar.set_writable()    # Set MyVariable to be writable by clients
    
    # Adding desired variables
    s = myobj.add_variable(idx, "s", 6.7)
    l = myobj.add_variable(idx, "l", 6.7)
    u = myobj.add_variable(idx, "u", 6.7)
    r = myobj.add_variable(idx, "r", 6.7)
    b = myobj.add_variable(idx, "b", 6.7)
    t = myobj.add_variable(idx, "t", 6.7)

    s.set_writable()
    l.set_writable()
    u.set_writable()
    r.set_writable()
    b.set_writable()
    t.set_writable()

    # starting!
    server.start()
    print("server is started")
    try:

        while True:
            time.sleep(0.1)
            s.set_value(position[0])
            l.set_value(position[1])
            u.set_value(position[2])
            r.set_value(position[3])
            b.set_value(position[4])
            t.set_value(position[5])
            #print(position[0])
    
    finally:
        #close connection, remove subcsriptions, etc
        server.stop()
