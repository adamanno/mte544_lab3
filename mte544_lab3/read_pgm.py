import matplotlib.pyplot as plt
import numpy as np

def read_pgm(f, neighbours, to_print = False):
    with open(f, 'rb') as pgmf:
        im = plt.imread(pgmf)
        
    cost_map = np.zeros(im.shape)

    for i in range(len(im)):
        for j in range(len(im[0])):
            if im[i,j] == 0:
                cost_map[i,j] = -2 ## Walls are 0
                for neighbour in neighbours:
                    cost_map[i+neighbour[0], j+neighbour[1]] = -1 ## Padding to the walls

    if to_print == True:
        print (cost_map)
        print (np.count_nonzero(cost_map))
        print (np.count_nonzero(cost_map == -1))
        print (np.count_nonzero(cost_map == -2))
        print (cost_map.size - np.count_nonzero(cost_map))
        print (cost_map.shape)
        
    else:
        return cost_map


f = '..\Office Map\map.pgm'
neighbours = [[0, -1], [0, 1], [-1, 0], [1, 0], [-1, -1], [-1, 1], [1, -1], [1, 1]]
cost_map = read_pgm(f, neighbours)