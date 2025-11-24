"""
UR5SimPullTool.py

Interfaz para el control simulado del UR5. 
- Conexión a polyscope
- Recepción observaciones relacionadas con el robot
- Directrices para el robot

Autor: Enrique de Antonio
"""
import rtde.rtde as rtde 
import rtde.rtde_config as rtde_config #instalación rtde pendiente

import numpy as np

from sim2real.IRobot.IRobot import IRobot
from sim2real.utils.data import list_to_setp, UR5SimRobotState

from pathlib import Path

import sys

class UR5Sim(IRobot):
    def __init__(self, ip: str, port: int, pos_init: np.array):
        
        # Asignación de variables iniciales
        super().__init__(ip, port)
        self.default_pos = pos_init #actualizar con valor inicial. Ver en código original.
        self.state= UR5SimRobotState(time= 0.0, joint_position=np.zeros(6), joint_velocities= np.zeros(6), online=False)
        
        # Nombre del archivo con la configuración del RTDE
        self.config_filename = Path(__file__).parent / "UR5Sim_config.xml"

        # Extracción de valores relevantes del fichero
        self.conf = rtde_config.ConfigFile(self.config_filename)
        state_names, state_types = self.conf.get_recipe("state")
        setp_names, setp_types = self.conf.get_recipe("setp")
        watchdog_name, watchdog_types = self.conf.get_recipe("watchdog")

        #Conexión con el robot
        self.con = rtde.RTDE(self.ip, self.port)

        try:
            self.con.connect()
        except TimeoutError:
            print("No se ha podido conectar al robot")
            con= None
            sys.exit()
        except ConnectionRefusedError:
            print("El robot ha rechazado la conexión")
            con= None
            sys.exit()
        
        if self.con != None:
            print("Conexión realizada.")
        
        # Configurar recetas
        self.con.send_output_setup(state_names, state_types)
        self.setp = self.con.send_input_setup(setp_names, setp_types)
        self.watchdog = self.con.send_input_setup(watchdog_name, watchdog_types)

        # Configuración registro inicial
        list_to_setp(self.setp, pos_init)
        self.initialized = False
        self.watchdog.input_int_register_0 = 1
        self.con.send(self.setp)
        self.con.send(self.watchdog)

        # Realizar configuración
        self.initialized = self.con.send_start()

    # Función para actualizar el estado del robot
    def get_state(self) -> UR5SimRobotState:
        # Inicializamos la clase tipo
        robotstate = UR5SimRobotState(joint_velocities=np.zeros(6), joint_position=np.zeros(6), online=False, time=0.0)
        # Mandamos una actualización del estado
        self.state = self.con.receive()
        # Comprobamos la actualización
        if self.state is None:
            # Devolvemos clase a 0 y con estado online negado
            robotstate=None
            return robotstate
        # Asignamos los valores
        if self.state.output_int_register_0 == 0:
            print("Elrobot ha rechazado la conexión. ")
            robotstate.online = False
            self._disconnect()
            return robotstate
        robotstate.joint_position= np.array(self.state.actual_q)
        robotstate.joint_velocities = np.array(self.state.actual_qd)
        robotstate.online = True
        # Entregamos el estado
        return robotstate

    def send_action(self, joint_pos: np.ndarray, robotstate: UR5SimRobotState):
        # Comprobamos si el estado esta activo
        if robotstate.online is False:
            print("No hay datos actualizados del Robot.")
            self.watchdog.input_int_register_0 = 0
            self.con.send(self.watchdog)
            return False
        else:
            self.watchdog.input_int_register_0 = 1
            self.con.send(self.watchdog)
            list_to_setp(self.setp, joint_pos)
            return self.con.send(self.setp)
        
    def _disconnect(self):
        self._reset_registers()
        self.watchdog.input_int_register_0 = 0
        self.con.send(self.watchdog)
        return self.con.disconnect()

    def _reset_registers(self):
        list_to_setp(self.setp, self.default_pos)
        self.con.send(self.setp)





