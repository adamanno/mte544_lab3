import numpy as np
import matplotlib.pyplot as plt
class node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position


def astar(maze, start, end):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""

    # Create start and end node
    start_node = node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = []

    # Add the start node
    open_list.append(start_node)

    # Loop until you find the end node
    while len(open_list) > 0:
        
        # Get the current node
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)

        # Found the goal, you can also implement what should happen if there is no possible path
        if current_node == end_node:
            # Complete here code to return the shortest path found
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1]

        # Complete here code to generate children, which are the neighboring nodes. You should use 4 or 8 points connectivity for a grid.
        children = []
        neighbours = [[0, -1], [0, 1], [-1, 0], [1, 0], [-1, -1], [-1, 1], [1, -1], [1, 1]]
        for single_neighbour in neighbours: # Adjacent squares
            # Get node position
            node_neighbour = (current_node.position[0] + single_neighbour[0], current_node.position[1] + single_neighbour[1])
            # Make sure within range and walkable
            
            if (0 > node_neighbour[0] or node_neighbour[0] > (len(maze) - 1) or 
               (0 > node_neighbour[1] or node_neighbour[1] > (len(maze[len(maze)-1]) -1)) or
               maze[node_neighbour[0]][node_neighbour[1]] != 0
            ):
                continue        
            # Create new node
            new_node = node(current_node, node_neighbour)
            # Append
            children.append(new_node)
            
        # Loop through children to update the costs
        for child in children:
            # Child is on the closed list
            for closed_child in closed_list:
                if child == closed_child:
                    break
            else:
                # Create the f, g, and h values, replace the 0s with appropriate formulations of the costs
                child.g = current_node.g + 1
                child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
                child.f = child.g + child.h

                # Complete here code to check whether to add a child to the open list
                open_list.append(child)

def main():

    # Load your maze here
    maze = []
    
    # This is an example maze you can use for testing, replace it with loading the actual map
    maze = [[0,   0,   0,   0,   1,   0, 0, 0, 0, 0],
            [0, 0.8,   1,   0,   1,   0, 0, 0, 0, 0],
            [0, 0.9,   1,   0,   1,   0, 1, 0, 0, 0],
            [0,   1,   0,   0,   1,   0, 1, 0, 0, 0],
            [0,   1,   0,   0,   1,   0, 0, 0, 0, 0],
            [0,   0,   0, 0.9,   0,   1, 0, 0, 0, 0],
            [0,   0, 0.9,   1,   1, 0.7, 0, 0, 0, 0],
            [0,   0,   0,   1,   0,   0, 0, 0, 0, 0],
            [0,   0,   0,   0, 0.9,   0, 0, 0, 0, 0],
            [0,   0,   0,   0,   0,   0, 0, 0, 0, 0]]

    
    
    # Define here your start and end points
    start = (0, 0)
    end = (6, 6)
    
    # Compute the path with your implementation of Astar
    path = np.asarray( astar(maze, start, end), dtype=float)
    maze_plot=np.transpose(np.nonzero(maze))

    plt.plot(maze_plot[:,0], maze_plot[:,1], 'o')
    
    if not np.any(path): # If path is empty, will be NaN, check if path is NaN
        print("No path found")
    else:
        plt.plot(path[:,0], path[:,1])
    plt.grid()
    plt.show()


if __name__ == '__main__':
    main()