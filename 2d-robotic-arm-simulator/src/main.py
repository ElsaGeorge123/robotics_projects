# src/main.py
from animator import ArmSimulator

def main():
    # lengths in meters (or arbitrary units)
    sim = ArmSimulator(L1=1.0, L2=0.8, steps=60,
                       joint_limits=[(-3.14, 3.14), (-3.14, 3.14)])
    sim.show()

if __name__ == "__main__":
    main()
