import time
import threading
from visualise import draw_robot, get_parameters
from kinematics.forward_kinematics import compute_fk

def run_fk():
    while True:
        x, y = compute_fk(get_parameters())
        time.sleep(0.1)

def main():
    t = threading.Thread(target=run_fk, daemon=True)
    t.start()
    draw_robot()

if __name__ == "__main__":
    main()