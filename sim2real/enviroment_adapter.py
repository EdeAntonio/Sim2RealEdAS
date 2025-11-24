"""
enviromen_adapter.py
---
Manejador principal del entorno a implementar mediante la política.

Esta clase tiene dos clases principales asignadas:
    PolicyController : Se encarga de manejar todo lo relacionado con la política. Desde cargarla y obtener los para-
    metros principales de la simulación hasta recibir las observaciones y computar la acción.
    IRobot : Clase a través de la cual se comunica con el robot, obteniendo las observaciones pertientes del robot
    y mandar las acciones generadas.

Además, para completar las observaciones se usará una clase observation hub, la cual tendrá asociada una serie de 
sensores para recibir el resto de inputs necesarios.

Autor: Enrique de Antonio
"""

import numpy as np

from abc import ABC, abstractmethod

from sim2real.PolicyController.policy_controller import PolicyController
from sim2real.utils import data
from sim2real.IRobot.IRobot import IRobot

from pathlib import Path

class EnviromentAdapter(ABC):

    state: data.EnvState

    dof_names= [
        "l_shoulder_pan_joint", 
        "l_shoulder_lift_joint", 
        "l_elbow_joint", 
        "l_wrist_1_joint", 
        "l_wrist_2_joint",
        "l_wrist_3_joint"
    ]

    def __init__(self, action_scale : float, action_size: int, model_path: str, robot: IRobot, frecuency: int = 125 ):
        self.controlador = PolicyController(self.dof_names)
        self.model_dir = Path(model_path)
        self.controlador.load_policy(
            self.model_dir / "policy.pt",
            self.model_dir / "env.yaml")
        self._action_scale = action_scale
        self._policy_counter = 0
        self._previous_action = np.zeros(action_size)

        self.robot = robot
        
        self.frecuency = frecuency

        self.has_joint_data = False
    
    @abstractmethod 
    def _compute_observation(self) -> np.ndarray:
        pass

    @abstractmethod
    def _update_state(self):
        pass 
    
    @abstractmethod
    def _compute_action(self, obs: np.ndarray) -> np.ndarray :
        pass

    def step(self) -> int:
        self._update_state()
        if self.has_joint_data == False:
            print("No hay datos del estado del robot. No se puede avanzar...\n")
            return -1
        if self.state.robot.online == False:
            self.robot._disconnect()
        self.obs = self._compute_observation()
        self.action: np.ndarray = self._compute_action(self.obs)
        success = self.robot.send_action(self.action, self.state.robot)
        if success == True :
            self._policy_counter += 1
            self._previous_action = self.action
            return 1
        else:
            print("Error inesperado en el paso.")
            return -1
        
        