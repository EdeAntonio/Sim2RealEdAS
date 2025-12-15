import torch
import numpy as np


# Carga la política TorchScript
policy = torch.jit.load("sim2real/policy_reach/policy.pt")
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
obs = [-0.1999,  0.3610, -0.2749, -0.3335,  0.3876, -0.4189, -3.1416,  1.3067,
         -2.1414, -2.8259,  3.9405, -6.2832,  0.3189, -0.0694,  0.1630,  0.6991,
         -0.1062,  0.6991,  0.1062, -2.0364,  0.8478, -0.7478, -0.9435,  1.1659,
         -2.4172]

# Obtener acción
action = compute_action(obs)
print("Action:", action)
