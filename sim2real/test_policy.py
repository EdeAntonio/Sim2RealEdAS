import torch
import numpy as np


# Carga la política TorchScript
policy = torch.jit.load("sim2real/policy_reach_2/policy.pt")
policy.eval()

# Función de prueba con observación
def compute_action(obs_vector):
    # convertir siempre a numpy array
    obs_np = np.array(obs_vector, dtype=np.float32)

    # convertir a tensor
    obs_t = torch.from_numpy(obs_np).unsqueeze(0)
    with torch.no_grad():
        action = policy(obs_t)
    return action.squeeze(0).numpy()

# Ejemplo de observación (25 elementos)
obs = np.zeros(25, dtype=np.float32)
obs = [
  0.312015653, 0.382867128, -0.168602541, -0.190106869,
  -0.155088425, -0.831667781, 0.0, 0.0,
  3.14989448, -1.15238678, 1.42854726, 0.289501637,
  -0.723364532, -4.60720015, 0.0, 0.0,
  0.164799616, -0.33471638, 0.212969497, -4.35871534e-08,
  0.997157812, 0.0753410608, -3.29326233e-09, 1.17599821,
  0.682768047, -0.172180608, -0.36113292, -0.394739419,
  -3.66745067, 3.14088202, -1.27364612
]

# Obtener acción
action = compute_action(obs)
print("Action:", action)
