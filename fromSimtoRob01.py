import sys
import socket
sys.path.insert(0, "..")
import time
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
)
import copy
from opcua import ua, Server
import numpy as np
import json
from simtorob import SimtoRob

def get_joint_values_from_file(filepath, variable_name):

    data = json.load(open(filepath))

    return np.asarray(data[variable_name])

filename1 = "joint_values_from_sim03.json"
filename2 = "joint_values_from_rob4.json"
robot: SimtoRob = SimtoRob(
        "192.168.255.200",
        [ControlGroupDefinition("robot", 0, 6, ["s", "l", "u", "r", "b", "t"])],
    )


robot.motion.start_servos()
robot.motion.start_trajectory_mode()

time.sleep(1)

status = robot.motion.check_motion_ready()
print("---------------------status of robot ready/not ready?: ")
print(status)

position = robot.state.joint_feedback(0).pos
#print("---------------------current position of robot joints: ")
position = list(position)
#print(position)

home = JointTrajPtFull(
                    groupno=0,
                    sequence=0,
                    valid_fields=ValidFields.TIME | ValidFields.POSITION | ValidFields.VELOCITY,
                    time=0,
                    pos = [0.0]*10,
                    vel = [0.0]*10,
                    acc = [0.0]*10
                )

threshold = 0.00003

check = list(np.array(position)-np.array(home.pos))
print("----check",check)

for item in check:
    
    sequence_nb = 0

    if abs(item) > threshold:
        timer = 5.0
        p0 = JointTrajPtFull(
                        groupno=0,
                        sequence=0,
                        valid_fields=ValidFields.TIME | ValidFields.POSITION | ValidFields.VELOCITY,
                        time=0,
                        pos = position,
                        vel = [0.0]*10,
                        acc = [0.0]*10
                    )
        robot.motion.send_joint_trajectory_point(p0)
        time.sleep(1)
        home = JointTrajPtFull(
                        groupno=0,
                        sequence=sequence_nb+1,
                        valid_fields=ValidFields.TIME | ValidFields.POSITION | ValidFields.VELOCITY,
                        time=timer,
                        pos = [0.0]*10,
                        vel = [0.0]*10,
                        acc = [0.0]*10
                    )
        robot.motion.send_joint_trajectory_point(home)
        sequence_nb +=1
        timer +=0.2
        time.sleep(5)
        print("robot has been moved to home pos and p0 updated", home.pos)
        break

    else:
        timer = 0
        home = JointTrajPtFull(
                        groupno=0,
                        sequence=sequence_nb,
                        valid_fields=ValidFields.TIME | ValidFields.POSITION | ValidFields.VELOCITY,
                        time=timer,
                        pos = robot.state.joint_feedback(0).pos,
                        vel = [0.0]*10,
                        acc = [0.0]*10
                    )
        robot.motion.send_joint_trajectory_point(home)
        sequence_nb +=1
        timer += 0.2
         

TrajPoints_fromSim = get_joint_values_from_file(filepath="joint_values_from_sim03.json", variable_name="joint_values")

for row in TrajPoints_fromSim:
    pos_vec = []
    zero_array = np.array([0,0,0,0])
    traj_point = np.append(row, zero_array)
    '''
    p0 = JointTrajPtFull(
                    groupno=0,
                    sequence=0,
                    valid_fields=ValidFields.TIME | ValidFields.POSITION | ValidFields.VELOCITY,
                    time=0,
                    pos = robot.state.joint_feedback(0).pos,
                    vel = [0.0]*10,
                    acc = [0.0]*10
                )
    robot.motion.send_joint_trajectory_point(p0)
    print("----------p0 updated", p0.pos)'''

    p1 = JointTrajPtFull(
                    groupno=0,
                    sequence=sequence_nb+1,
                    valid_fields=ValidFields.TIME | ValidFields.POSITION | ValidFields.VELOCITY,
                    time=timer,
                    pos = np.deg2rad(traj_point),
                    vel = [0.0]*10,
                    acc = [0.0]*10
                )
    robot.motion.send_joint_trajectory_point(p1)
    sequence_nb +=1
    timer += 0.2
    time.sleep(0.2)
    position_vector = robot.state.joint_feedback(0).pos
    print(position_vector)
    new_s = position_vector[0]
    new_l = position_vector[1]
    new_u = position_vector[2]
    new_r = position_vector[3]
    new_b = position_vector[4]
    new_t = position_vector[5]

    vec = [new_s, new_l, new_u, new_r, new_b, new_t]
    data = json.load(open(filename2))
    data_vec = data["joint_values"]
    data_vec.append(vec)
    data["joint_values"] = data_vec 
        
    with open(filename2, "w") as f:
        json.dump(data, f, indent=2)
                
    pos_vec.append(vec)

