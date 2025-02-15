#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from math import pi


def visualization_orient(csv_name: str):
    # load csv file and plot trajectory
    _, ax = plt.subplots(1)
    trajectory = np.loadtxt(csv_name, delimiter=",")
    plt.plot(list(range(0, len(trajectory))), trajectory, linewidth=2)
    plt.xlim(0, len(trajectory))
    plt.ylim(-pi, pi)
    plt.minorticks_on()
    plt.grid(which="both")
    plt.xlabel("x (unit)")
    plt.ylabel("y (rad)")
    plt.show()


def visualization_pos(csv_name: str):
    # load csv file and plot trajectory
    _, ax = plt.subplots(1)
    trajectory = np.loadtxt(csv_name, delimiter=",")
    plt.plot(list(range(0, len(trajectory))), trajectory, linewidth=2)
    plt.xlim(0, len(trajectory))
    plt.ylim(-0.39, 0.8)

    plt.minorticks_on()
    plt.grid(which="both")
    plt.xlabel("x (unit)")
    plt.ylabel("z (m)")
    plt.show()


if __name__ == "__main__":
    visualization_orient("trajectory1.csv")
    visualization_orient("trajectory2.csv")
    visualization_pos("trajectory3.csv")
