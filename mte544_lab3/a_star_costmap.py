import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSProfile
import numpy as np
import cv2
import matplotlib.pyplot as plt
from nav_msgs.msg import OccupancyGrid

class getCostMap (Node):

    def __init__(self):
        super().__init__('getCostMap')
        self.get_logger().info('getCostMap started successfully. Starting.') 
        # Set QoS profile
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribe to cost map
        self.subscription = self.create_subscription(OccupancyGrid,'/global_costmap/costmap',self.costmap,qos_profile=qos_profile)
        self.subscription  # prevent unused variable warning
        self.data = []
        self.gotmap = False

    def costmap (self, data):
        if (not self.gotmap):
            self.data = data.data
            print("hi") # To check if the code gets to this point
            self.resolution = 0.05 #meters/pixel
            self.origin = np.array([-5.75, -3.72]) # Translation of map to map frame
            self.maze = np.empty((data.info.height, data.info.width), int) # 1D array in row-major format
            self.maze = np.reshape(self.data, (-1, data.info.width)) # Reshape into 2D array with same dimensions as map
            scale_percent = 5 # percent of original size to scale the array down to reduce computation

            # Scaled dimensions, keeping aspect ratio intact
            width = int(data.info.width* scale_percent / 100)
            height = int(data.info.height * scale_percent / 100)
            dim = (width, height)
            
            # resize map/occupancy grid
            self.maze = cv2.resize(self.maze.astype('float32'), dim, interpolation = cv2.INTER_AREA)

            #Find Occupancy Grid from Cost Map
            self.maze[self.maze < 63] = 0 # Free space
            self.maze[self.maze > 62] = 1 # Occupied Space
            
            # For display the map if need be
            # img = self.maze
            # self.maze[self.maze > 0] = 205
            # cv2.imshow("maze", img)
            # cv2.waitKey(0)
            # cv2.destroyAllWindows()

            # index * resolution + origin = cart_coordinate
            # (cart_coordinate - x_origin) / resolution = index
            # Determining the start and end coordinates by transforming it from global frame to pixel/array indices using the second equation above.
            start_cart = np.array([0, -0.08])
            start = (np.rint((start_cart - self.origin)/(self.resolution / (scale_percent / 100)))).astype(int)
            start[1]=(height-1)-start[1]
            start = start.tolist()
            end_cart = np.array([3, 1.9])
            end = (np.rint((end_cart - self.origin)/(self.resolution / (scale_percent / 100)))).astype(int)
            end[1]=(height-1)-end[1]
            end = end.tolist()

            # Print value of start and end points to check it is free space (aka 0) so that A* works
            print (self.maze[start[1]][start[0]])
            print (self.maze[end[1]][end[0]])
                        
            # Compute the path with implementation of A#
            path = np.asarray( astar(self.maze, start, end), dtype=float)

            # Convert the path to global frame
            path = path * (self.resolution/(scale_percent/100)) + self.origin
            
            # Write the path coordinatees to a .txt file to be imported into movement controller script later
            with open ('/home/hari/lab3/src/mte544_lab3/mte544_lab3/path.txt', 'w') as f:
                f.write(path)

            #Set true so it only runs once
            self.gotmap = True

class node():
    """A node class for A* Pathfinding"""

    # Initalizes a node with its parent node (aka where it came from, its current position, and its costs)
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0 # Cost
        self.h = 0 # Heuristic
        self.f = 0 # Cost + Heuristic

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
        #Finding the node in the open list with the lowest cost, which is will be the current node
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)

        # Found the goal, you can also implement what should happen if there is no possible path
        if current_node.position == end_node.position:
            path = []
            current = current_node

            # Trace through the closed list to find the path from the end node to the start node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1]

        # Generate children, which are the neighboring nodes. Uses 8 points connectivity for a grid.
        children = []
        #Neighbors in the following directions: South, North, West, East, South-West, North-West, South-East, North-West
        neighbours = [[0, -1], [0, 1], [-1, 0], [1, 0], [-1, -1], [-1, 1], [1, -1], [1, 1]]
        for single_neighbour in neighbours: # Adjacent squares
            # Get node position
            node_neighbour = (current_node.position[0] + single_neighbour[0], current_node.position[1] + single_neighbour[1])
            # Make sure within range and walkable
            
            if (0 > node_neighbour[0] or node_neighbour[0] > (len(maze) - 1) or 
               (0 > node_neighbour[1] or node_neighbour[1] > (len(maze[len(maze)-1]) -1)) or
               maze[node_neighbour[0]][node_neighbour[1]] != 0
            ):
                continue # Go to next neighbour if this neighbour is invalid
            #if valid
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
                # Calculate the cost only if not in closed list
                child.g = current_node.g + 1
                child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
                child.f = child.g + child.h

                # Complete here code to check whether to add a child to the open list
                open_list.append(child)

                # for open_child in open_list:
                #     if child.position == open_child.position and child.f<open_child.f:
                #         open_child=child
                #         break
                #     elif child.position == open_child.position and child.f>open_child.f:
                #         break
                # else:
                #     # Add current child to list only if not already in open list
                #     open_list.append(child)

def main(args=None):
    rclpy.init(args=args)  
    map = getCostMap()

    # # This is an example maze you can use for testing, replace it with loading the actual map
    # maze = [[0,   0,   0,   0,   1,   0, 0, 0, 0, 0],
    #         [0, 0.8,   1,   0,   1,   0, 0, 0, 0, 0],
    #         [0, 0.9,   1,   0,   1,   0, 1, 0, 0, 0],
    #         [0,   1,   0,   0,   1,   0, 1, 0, 0, 0],
    #         [0,   1,   0,   0,   1,   0, 0, 0, 0, 0],
    #         [0,   0,   0, 0.9,   0,   1, 0, 0, 0, 0],
    #         [0,   0, 0.9,   1,   1, 0.7, 0, 0, 0, 0],
    #         [0,   0,   0,   1,   0,   0, 0, 0, 0, 0],
    #         [0,   0,   0,   0, 0.9,   0, 0, 0, 0, 0],
    #         [0,   0,   0,   0,   0,   0, 0, 0, 0, 0]]

    
    
    # # Define here your start and end points
    # start = (0, 0)
    # end = (6, 6)
    
    # # Compute the path with your implementation of Astar
    # path = np.asarray( astar(maze, start, end), dtype=float)
    # maze_plot=np.transpose(np.nonzero(maze))

    # plt.plot(maze_plot[:,0], maze_plot[:,1], 'o')
    
    # if not np.any(path): # If path is empty, will be NaN, check if path is NaN
    #     print("No path found")
    # else:
    #     plt.plot(path[:,0], path[:,1])
    # plt.grid()
    # plt.show()
    

    rclpy.spin(map)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    map.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()