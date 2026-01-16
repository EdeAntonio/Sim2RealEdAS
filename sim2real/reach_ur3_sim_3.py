"""
PullObjectToolUR5Sim.py

Clase heredada de EnviromentAdapter. Especifica dicha clase para el problema de
arrastre de objeto con herramienta.
"""

import numpy as np

from sim2real.enviroment_adapter import EnviromentAdapter 

from sim2real.ObservationHub.sensor_hub import SensorHub
from sim2real.IRobot.UR5Sim import UR5Sim
from sim2real.utils.data import EnvState

import time


import argparse
parser = argparse.ArgumentParser(description="Sim2Real: Ejercicio de arrastre objeto con herramienta.")
parser.add_argument("--policy_path", type=str, default=None, help="Direccion de la carpeta con la política y la configuración.")
parser.add_argument("--target", type=float, nargs="+", default=None, help="Posicion objetivo de alcance")
args  = parser.parse_args()

class ReachUR3Sim (EnviromentAdapter):
    dof_names= [
        "shoulder_pan_joint", 
        "shoulder_lift_joint", 
        "elbow_joint", 
        "wrist_1_joint", 
        "wrist_2_joint",
        "wrist_3_joint"
    ]

    def __init__(self):
        super().__init__(action_scale = 0.5, action_size = 6, model_path=args.policy_path, robot = UR5Sim("192.168.1.140", 30004, pos_init=[-1.57, -1.744, 1.57, -1.573, -1.573, -1.923]))
        self.default_pos: np.ndarray = self.robot.default_pos
        self.mode = 1
        self.state = EnvState(robot = None, object_pos= None, tool_pos= None)
        if len(args.target) == 7:
            self.target=np.zeros(7)
            self.target = args.target
            print(f"Objetivo fijado en: {self.target}")
        elif len(args.target) == 6:
            print(args.target)
            self.target = np.zeros(7)
            self.target[:3] = args.target[:3]
            self.target[3:] = self.get_quaternion_from_euler(roll=args.target[3], pitch=args.target[4], yaw=args.target[5])
            print(f"Objetivo fijado en: {self.target}")
        else:
            print("Dimensión incorrecta. Debe darse como posicion (3) + cuaternio (4) o posicion (3) + rotacion (3)")

    
    def _update_state(self):
        self.state.robot = self.robot.get_state()
        if self.state.robot == None:
            print("Problemas con el estado del Robot. Estado vacío...\n")
            self.has_joint_data = False
            return None
        self.has_joint_data = True
        self.state.object_pos = SensorHub.object_position_test()
        self.state.tool_pos = SensorHub.tool_position_test(self.mode)
        return self.state

    def _compute_observation(self) -> np.ndarray:
        obs = np.zeros(25)
        obs[:6] = self.state.robot.joint_position - self.default_pos
        obs[6:12] = self.state.robot.joint_velocities
        obs[12:19] = self.target
        obs[19:25] = self._previous_action
        return obs

    def _compute_action(self, obs : np.ndarray) -> np.ndarray:
        action = self.controlador._compute_action(obs)
        return action[:6]

    def get_quaternion_from_euler(self, roll: float, pitch: float, yaw:float):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    
        return qw, qx, qy, qz

if __name__ == "__main__":
    entorno : ReachUR3Sim = ReachUR3Sim()
    online = False
    while online == False:
        if entorno._update_state() != None:
            online = True
            start_time= time.time()
        time.sleep(0.125)
    print("En ejecución...")
    while (online == True):
        regreso = entorno.step()
        if regreso < 0:
            count_time = time.time() - start_time
            print("Error detectado... Fin simulación. Tiempo: {:02}".format(count_time))
            online= False
        time.sleep(0.125)

