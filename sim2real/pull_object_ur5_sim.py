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

class POTUR5Sim (EnviromentAdapter):
    def __init__(self):
        super().__init__(action_scale = 0.5, action_size = 6, model_path=args.policy_path, robot = UR5Sim("192.168.1.103", 30004, pos_init=[-3.86, 0.3, -2.24, 0.91, 2.22, -1.75]))
        self.default_pos: np.ndarray = self.robot.default_pos
        self.mode = 1
        self.state = EnvState(robot = None, object_pos= None, tool_pos= None)
    
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
        obs = np.zeros(52)
        obs[8:14] = self.state.robot.joint_position - self.default_pos
        obs[24:30] = self.state.robot.joint_velocities
        obs[32:35] = self.state.tool_pos
        obs[35:38] = self.state.object_pos
        obs[38:44] = (0.4, 0.4, 0, 0, 0, 0)
        obs[44:50] = self._previous_action
        obs[50:55] = (0, 0)
        return obs

    def _compute_action(self, obs : np.ndarray) -> np.ndarray:
        action = self.controlador._compute_action(obs)
        return action[:6]

if __name__ == "__main__":
    entorno : POTUR5Sim = POTUR5Sim()
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


    