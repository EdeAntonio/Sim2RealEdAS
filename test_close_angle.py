
import math

import argparse
parser = argparse.ArgumentParser(description="Sim2Real: Ejercicio de arrastre objeto con herramienta.")
parser.add_argument("--current", type=float, default=None)
parser.add_argument("--target", type=float, default=None)
args  = parser.parse_args()


current = float(args.current) * 2.0 * math.pi/360.0
target = float(args.target) * 2.0 * math.pi/360.0

current_pol = (current+math.pi)%(2*math.pi)-math.pi

diff= (target - current + math.pi) % (2*math.pi) - math.pi
target_real = current + diff
target_real_grad = target_real*360/(2*math.pi)

print(f"Rad: {target_real}")
print(f"Ang: {target_real_grad}")
print(f"Rad: {current_pol}")