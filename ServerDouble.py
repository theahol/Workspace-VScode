import sys
import csv
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

robot_pos_file = "serverDouble_robot_test1.json"
simulation_pos_file = "serverDouble_sin_test1.json"

robot: Moto = Moto(
        "192.168.255.200",
        [ControlGroupDefinition("robot", 0, 6, ["s", "l", "u", "r", "b", "t"])],
    )

#connect to physical robot and prepare for movement 
robot.motion.start_servos()
robot.motion.start_trajectory_mode()
check_ready = robot.motion.check_motion_ready()
print("robot ready?..............")
print(check_ready)
time.sleep(3)


def disconnect(self):
    client.close()

if __name__ == "__main__":

    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # setup our server
    server = Server()
    server.set_endpoint("opc.tcp://localhost:4840/motopcua/server/")

    # setup our own namespace, not really necessary but should as spec
    uri = "http://server.motopcua.github.io"
    idx = server.register_namespace(uri)

    # get Objects node, this is where we should put our nodes
    objects = server.get_objects_node()

    # populating our address space
    myobj = server.nodes.objects.add_object(idx, "Moto")
    myvar = myobj.add_variable(idx, "MyVariable", 6.7)

    myvar.set_writable()    # Set MyVariable to be writable by clients

    # Adding desired variables from server to simulation
    s = myobj.add_variable(idx, "s", 6.7)
    l = myobj.add_variable(idx, "l", 6.7)
    u = myobj.add_variable(idx, "u", 6.7)
    r = myobj.add_variable(idx, "r", 6.7)
    b = myobj.add_variable(idx, "b", 6.7)
    t = myobj.add_variable(idx, "t", 6.7)
    
    # set joints writable 
    s.set_writable()
    l.set_writable()
    u.set_writable()
    r.set_writable()
    b.set_writable()
    t.set_writable()

    #Adding desired variables from simulation to server 
    joint_S = myobj.add_variable(idx, "S1", 6.7)
    joint_L = myobj.add_variable(idx, "L1", 6.7)
    joint_U = myobj.add_variable(idx, "U1", 6.7)
    joint_R = myobj.add_variable(idx, "R1", 6.7)
    joint_B = myobj.add_variable(idx, "B1", 6.7)
    joint_T = myobj.add_variable(idx, "T1", 6.7)

    joint_S.set_writable()
    joint_L.set_writable()
    joint_U.set_writable()
    joint_R.set_writable()
    joint_B.set_writable()
    joint_T.set_writable()

    # starting!
    server.start()
    print("server is started")
    try:
        pos_vec_fromRob = []
        pos_vec = []
        index = 0
        timer = 10
        #position_vector_to_sim = []
        while True:
            
            
            S_updated = joint_S.get_value()
            L_updated = joint_L.get_value()
            U_updated = joint_U.get_value()
            R_updated = joint_R.get_value()
            B_updated = joint_B.get_value()
            T_updated = joint_T.get_value()
            

            if S_updated != 6.7 or L_updated != 6.7 or U_updated != 6.7 or R_updated != 6.7 or B_updated != 6.7 or T_updated != 6.7:
                vec = [S_updated, L_updated, U_updated, R_updated, B_updated, T_updated]
                print("########### Loop {}".format(index))

                data = json.load(open(simulation_pos_file))
                data_vec = data["joint_values"]
                data_vec.append(vec)
                data["joint_values"] = data_vec 
                        
                with open(simulation_pos_file, "w") as f:
                    json.dump(data, f, indent=2)

                pos_vec.append(vec)
                print(pos_vec)
                
                position = pos_vec[index]

                zero_array = np.array([0,0,0,0])
                trajectory_point = list(np.append(position, zero_array))
                print('trajectory point sent to robot; ', trajectory_point)
                        
                
                                
                #pos_vec.append(vec) 
                
                #print("\n--> Trying to update p0 for robot")
                #position1 = robot.state.joint_feedback(0).pos
                #currentpos pos= position[index-1] bortsett fra når index = 0, må bruke home
                # if index=0, bruk p0=home. else bruk pos=position[index-1]
                #events eller callbacks i opcua 
                print(index)
                if index == 0:
                    currentPosition = JointTrajPtFull(
                            groupno=0,
                            sequence=0,
                            valid_fields=ValidFields.TIME | ValidFields.POSITION | ValidFields.VELOCITY,
                            time=0,
                            pos = robot.state.joint_feedback(0).pos,
                            vel = [0.0]*10,
                            acc = [0.0]*10
                        )
                    robot.motion.send_joint_trajectory_point(currentPosition)
                    #print("----------currentPos updated", currentPosition.pos)
                    
                    time.sleep(0.1)
                    #print("\n--> Trying to update p1 for robot")
                    newPosition = JointTrajPtFull(
                            groupno=0,
                            sequence=index+1,
                            valid_fields=ValidFields.TIME | ValidFields.POSITION | ValidFields.VELOCITY,
                            time=timer,
                            pos = [0.0]*10,
                            vel = [0.0]*10,
                            acc = [0.0]*10
                        )
                    robot.motion.send_joint_trajectory_point(newPosition)
                    time.sleep(0.2)
                    print("----------newPos updated", newPosition.pos)
                    index = 1
                    #time.sleep(10)
                    timer +=0.2
                    print('timer from IF: ', timer)
                
                else:

                    #print("\n--> Trying to update p1 for robot")
                    newPosition = JointTrajPtFull(
                                groupno=0,
                                sequence=index+1,
                                valid_fields=ValidFields.TIME | ValidFields.POSITION | ValidFields.VELOCITY,
                                time=timer,
                                pos = np.deg2rad(trajectory_point),
                                vel = [0.0]*10,
                                acc = [0.0]*10
                        )
                    robot.motion.send_joint_trajectory_point(newPosition)

                    time.sleep(0.3)
                    print("----------newPos updated", newPosition.pos)

                    #problemer med å lese posisjon fra fysisk robot. Koden kjører for fort så den klarer ikke lese 
                    #fordi roboten har ikke beveget seg enda. 
                    
                    #check = robot.robot_status.in_motion()
                    #if check == true:
                        #for very small time increments 

                    position2 = robot.state.joint_feedback(0).pos
                    rad = np.rad2deg(position2)
                    #position_vector_to_sim.append(position2)
                    #print('positions read from robot:', position2)
                    #tosim = position_vector_to_sim[teller]
                    #print('Positon of the robot sent to twin', tosim)

                    s.set_value(rad[0])
                    l.set_value(rad[1])
                    u.set_value(rad[2])
                    r.set_value(rad[3])
                    b.set_value(rad[4])
                    t.set_value(rad[5])
                    
                    position_vector = robot.state.joint_feedback(0).pos
                    #print(position_vector)
                    s_from_rob = position_vector[0]
                    l_from_rob = position_vector[1]
                    u_from_rob = position_vector[2]
                    r_from_rob = position_vector[3]
                    b_from_rob = position_vector[4]
                    t_from_rob = position_vector[5]

                    
                    vec_fromRob = [s_from_rob, l_from_rob, u_from_rob, r_from_rob, b_from_rob, t_from_rob]
                    data2 = json.load(open(robot_pos_file))
                    data_vec2 = data2["joint_values"]
                    data_vec2.append(vec_fromRob)
                    data2["joint_values"] = data_vec2 
                        
                    with open(robot_pos_file, "w") as f:
                        json.dump(data2, f, indent=2)
                                
                    pos_vec_fromRob.append(vec_fromRob)
                    print(pos_vec_fromRob)

                    index += 1
                    timer += 0.2
                    print('Timer from ELSE:', timer)
                    
            
                

    finally:
        #close connection, remove subcsriptions, etc
        server.stop()
