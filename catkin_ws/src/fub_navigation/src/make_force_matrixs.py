#!/usr/bin/env python2
import numpy as np
from scipy.spatial import KDTree

import path_parser

map_size_x = 600  # cm
map_size_y = 430  # cm
resolution = 1  # cm
lookahead_offset = 25  # *resolution cm = for example 5 *10= 50 cm //Dynamic: np.int(2 + (8/(8*dist+1)))
matrix = np.zeros((map_size_x / resolution, map_size_y / resolution, 2), dtype='f')


def main(map_file):
    xy = np.array(list(path_parser.read_points(map_file)))
    x, y = xy.T

    tree = KDTree(xy)

    # fig = plt.figure(figsize=(12, 10), facecolor='w')
    # plt.plot(x, y, ':o', markersize=2)

    # plt.tight_layout()

    def show_nearest(target):
        dist, index = tree.query(target)
        global lookahead_offset
        # lookahead_offset = 10#np.int(2 + (8/(8*dist+1)))
        lookahead_target = xy[(index + lookahead_offset) % len(xy)]

        x1, y1 = target
        x3, y3 = lookahead_target

        # plt.scatter(*target, color='r')
        # plt.scatter(*xy[index], color='g')
        # ax = plt.axes()
        # ax.arrow(x1, y1, (x3-x1)/5, (y3-y1)/5 , head_width=0.01, head_length=0.01, fc='k', ec='k')
        # plt.scatter(*lookahead_target, color='m')
        # plt.show(block=False)
        global matrix
        x_index = np.int(x1 * (100.0 / resolution))
        y_index = np.int(y1 * (100.0 / resolution))

        matrix[x_index, y_index, 0] = x3 - x1
        matrix[x_index, y_index, 1] = y3 - y1

    print('please wait ...')
    for x in range(0, map_size_x / resolution):
        for y in range(0, map_size_y / resolution):
            show_nearest(((resolution / 100.0) * x, (resolution / 100.0) * y))

    np.save('matrix' + str(lookahead_offset) + 'cm_lane2.npy', matrix)
    print('matrix is saved.')
    # plt.show()


if __name__ == '__main__':
    main('new_map_loop2.txt')
