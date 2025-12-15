"""
SensorHub: Clase para la creación de la conexión con los sensores. En este caso al
ser puramente para el ensayo de pruebas vamos a realizar una clase básica para generar
puntos en el espacio

"""

import numpy as np

class SensorHub:

    def __init__(self):
        pass

    @staticmethod
    def object_position_test():
        object_position = np.array([0.6, -0.4, 0.055])
        return object_position
    
    @staticmethod
    def tool_position_test(mode: int):
        if mode == 1:
            tool_position = np.array([0.143, -0.3161, 0.008])
        return tool_position

        