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
        super().__init__(action_scale = 0.5, action_size = 6, model_path=args.policy_path, robot = UR5Sim("192.168.1.140", 30004, pos_init=[0, -1.712, 1.712, 0, 0, 0]))
        self.default_pos: np.ndarray = self.robot.default_pos
        self.mode = 1
        self.state = EnvState(robot = None, object_pos= None, tool_pos= None)
        self.target_ee_pose = np.array([0.20, -0.20,  0.25,  0.6991, -0.1062,  0.6991,  0.1062], dtype=np.float32)

    
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
        obs[12:19] = self.target_ee_pose
        obs[19:25] = self._previous_action
        return obs

    def _compute_action(self, obs : np.ndarray) -> np.ndarray:
        action = self.controlador._compute_action(obs)
        return action[:6]

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