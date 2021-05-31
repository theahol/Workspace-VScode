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
import os

class SimtoRob(Moto):
    def __init__(self, ip, control_group_defs):
        super().__init__(ip, control_group_defs)
        while self.state.joint_feedback(0) == None:
            pass
        self.p0 = JointTrajPtFull(
            groupno=0,
            sequence=0,
            valid_fields=ValidFields.TIME | ValidFields.POSITION | ValidFields.VELOCITY,
            time=0,
            pos = self.state.joint_feedback(0).pos,
            vel = self.state.joint_feedback(0).vel,
            acc=self.state.joint_feedback(0).acc
        )

    def update_trajpoint_p0(self):
        self.p0 = JointTrajPtFull(
            groupno=0,
            sequence=0,
            valid_fields=ValidFields.TIME | ValidFields.POSITION | ValidFields.VELOCITY,
            time = 0,
            pos = self.state.joint_feedback(0).pos,
            vel=[0.0]*10,
            acc=[0.0]*10
        )
        #self.motion.send_joint_trajectory_point(self.p0)

    def drive_to_home(self):
        self.update_trajpoint_p0()
        home = JointTrajPtFull(
            groupno=0,
            sequence=1,
            valid_fields=ValidFields.TIME | ValidFields.POSITION | ValidFields.VELOCITY,
            time = 10,
            pos = [0.0]*10,
            vel=[0.0]*10,
            acc=[0.0]*10
        )
        #self.motion.send_joint_trajectory_point(home)
    
    @staticmethod
    def make_traj_point(pos: List[float], 
                time: int,
                groupno = 0,
                sequence= 1,
                valid_fields=ValidFields.TIME | ValidFields.POSITION | ValidFields.VELOCITY,
                vel = [0.0]*10,
                acc = [0.0]*10
                ) -> JointTrajPtFull:
        for _ in range(4):
            pos.append(0.0)
            point = JointTrajPtFull(groupno, sequence, valid_fields, time, np.deg2rad(pos), vel, acc)
            return(point)
