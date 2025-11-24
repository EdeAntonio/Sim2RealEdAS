"""
dataclasses.py

En este fichero se declaran las dataclasse que se necesitan para el proyecto. De este modo se agrupan las variables y se devuelven paquetes capaces de ser leidos por todas las clases

"""
from dataclasses import dataclass
import numpy as np

@dataclass
class RobotState:
    # Representa el estado de cualquier robot. De esta clase heredan luego los ditintos tipos de robot
    time: float

@dataclass
class UR5SimRobotState(RobotState):
    joint_position: np.ndarray
    joint_velocities: np.ndarray
    online: bool

def setp_to_list(sp, size: int)->np.ndarray:
    sp_list = []
    for i in range(0, size):
        sp_list.append(sp.__dict__["input_double_register_%i" % i])
    return np.array(sp_list)

def list_to_setp(sp, list: np.ndarray):
    for i in range(0, len(list)):
        sp.__dict__["input_double_register_%i" % i] = list[i]

@dataclass
class EnvState:
    # Representa el estado de cualquier robot. De esta clase heredan luego los ditintos tipos de robot
    robot: UR5SimRobotState
    object_pos : np.ndarray
    tool_pos : np.ndarray
    #SensorHub

