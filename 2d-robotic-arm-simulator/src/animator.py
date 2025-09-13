# src/animator.py
import numpy as np
import matplotlib.pyplot as plt
from forward_kinematics import forward_kinematics
from inverse_kinematics import inverse_kinematics_2link

class ArmSimulator:
    def __init__(self, L1=1.0, L2=0.8, steps=60, joint_limits=None):
        self.Ls = [L1, L2]
        self.thetas = [0.0, 0.0]   # current joint angles (rad)
        self.steps = steps         # frames for interpolation
        self.joint_limits = joint_limits  # e.g. [(-pi,pi), (-pi,pi)]
        self.fig, self.ax = plt.subplots(figsize=(6,6))
        self.line, = self.ax.plot([], [], marker='o')
        self.target_dot, = self.ax.plot([], [], marker='x', markersize=8)
        Lsum = sum(self.Ls)
        self.ax.set_xlim(-Lsum-0.1, Lsum+0.1)
        self.ax.set_ylim(-Lsum-0.1, Lsum+0.1)
        self.ax.set_aspect('equal', 'box')
        self.ax.grid(True)
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.draw_arm(self.thetas)

    def draw_arm(self, thetas):
        positions = forward_kinematics(self.Ls, thetas)
        xs = [p[0] for p in positions]
        ys = [p[1] for p in positions]
        self.line.set_data(xs, ys)
        self.fig.canvas.draw_idle()

    def on_click(self, event):
        if event.inaxes != self.ax:
            return
        x, y = event.xdata, event.ydata
        sols = inverse_kinematics_2link(x, y, self.Ls[0], self.Ls[1])
        if sols is None:
            print(f"[unreachable] target ({x:.2f}, {y:.2f}) outside workspace")
            # optional: mark unreachable briefly
            self.target_dot.set_data([x], [y])
            self.fig.canvas.draw_idle()
            return
        # choose solution with least joint-change from current pose
        cur = np.array(self.thetas)
        candidates = [np.array(s) for s in sols]
        dists = [np.linalg.norm(c - cur) for c in candidates]
        goal = candidates[int(np.argmin(dists))]
        # enforce joint limits if provided
        if self.joint_limits:
            for i, (low, high) in enumerate(self.joint_limits):
                goal[i] = np.clip(goal[i], low, high)
        # animate
        self.target_dot.set_data([x], [y])
        self.animate_to(goal)

    def animate_to(self, goal):
        start = np.array(self.thetas)
        goal = np.array(goal)
        for t in np.linspace(0, 1, self.steps):
            interp = start + (goal - start) * t
            self.draw_arm(interp)
            plt.pause(0.01)
        self.thetas = list(goal)
        self.draw_arm(self.thetas)

    def show(self):
        self.draw_arm(self.thetas)
        plt.title("Click anywhere to move the end-effector (2-link arm)")
        plt.show()
