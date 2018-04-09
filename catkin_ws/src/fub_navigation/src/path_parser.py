import numpy as np
import os


def read_points(map_file, offset_x=0.0, offset_y=0.0):
    """
    Reads a file with the map data in the RNDF Format
    :return: generator of x, y position tuples
    """

    # detect waypoints x.y.z and store latitude / longitude as x / y
    with open(map_file) as m_file:
        for line in m_file:
            if line.startswith('1.1.'):
                x, y = line.split('\t')[1:3]
                yield float(x) + offset_x, float(y) + offset_y
    with open(map_file) as m_file:
        for line in m_file:
            if line.startswith('1.2.'):
                x, y = line.split('\t')[1:3]
                yield float(x) + offset_x, float(y) + offset_y


def relative_filename(name):
    return os.path.join(os.path.dirname(__file__), name)


def build_kdtree():
    """
    helper method for reading nodes and building a KDTree from them
    :return: nodes and the KDTree
    """
    from scipy.spatial import KDTree

    map_file = relative_filename('sample_map_origin_map.txt')
    nodes = np.array(list(read_points(map_file)))
    tree = KDTree(nodes)
    return nodes, tree


def read_speeds(variant='norm_speeds'):
    speed_file = relative_filename(variant + '.npz')
    with np.load(speed_file) as pickle:
        return pickle.f.arr_0
