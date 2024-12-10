import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from lattice_planner.lattice_graph import LatticeGraph
from lattice_planner.utils import ObstaclesGrid
from lattice_planner.utils import *
from trajectory_generator.traj_generation import TrajGenerator
from tracking_controller.pid import PIDTrajectoryTrackingNode
import matplotlib.pyplot as plt
from lattice_planner.utils import *

def main(args=None):
    # Initialize the graph and trajectory generator
    graph = LatticeGraph()
    traj_generator = TrajGenerator()

    # Define the graph dimensions and scaling
    n_rows = 10
    n_cols = 10
    lattice_cell_size = 10
    scaler = 3.0
    graph.initialise_graph(n_rows=n_rows, n_cols=n_cols, lattice_cell_size=lattice_cell_size)

    # Define the start and goal positions with orientation
    s = (1, 8, 90)  # Start position: row, col, angle
    g = (8, 2, 270)  # Goal position: row, col, angle

    # Initialize obstacle grids
    obs = ObstaclesGrid(map_size=(n_rows * lattice_cell_size, n_cols * lattice_cell_size))
    obs_plot = ObstaclesGrid(map_size=(int(n_rows * lattice_cell_size / scaler), int(n_cols * lattice_cell_size / scaler)))

    # Add obstacles to the map
    obs.map[25:35, 45:56] = True
    obs.map[67:89, 57:76] = True
    obs.map[50:55, 80:89] = True
    obs.map[20:60, 25:35] = True

    # Add scaled obstacles for plotting
    obs_plot.map[int((25 / scaler)):int((35 / scaler)), int((45 / scaler)):int((56 / scaler))] = True
    obs_plot.map[int((67 / scaler)):int((89 / scaler)), int((57 / scaler)):int((76 / scaler))] = True
    obs_plot.map[int((50 / scaler)):int((55 / scaler)), int((80 / scaler)):int((89 / scaler))] = True
    obs_plot.map[int((20 / scaler)):int((60 / scaler)), int((25 / scaler)):int((35 / scaler))] = True

    # Update the graph with obstacle information
    graph.update_obstacles(obs)

    # Solve the graph to find a path from start to goal
    path = graph.solve(s, g, graph._graph._vert_list, graph._graph._adjacency_matrix, graph._graph._edge_dict)

    # Interpolate the path for smoothness
    path_interpolated = traj_generator.path_interpolation(path, graph, lattice_cell_size, 10)

    # Resample the interpolated path to generate a trajectory
    result = traj_generator.resample_path(path_interpolated)

    # Print the number of states in the trajectory
    print("trajectory length = ", len(result.states))

    # Write the trajectory to a YAML file
    write_result_to_yaml(result, 'solution.yaml')

    # Visualization section
    # Plot the obstacle map and trajectory
    fig = plot_map(obs_plot, graph, lattice_cell_size)
    x = []
    y = []
    for i in range(len(result.states)):
        x.append(result.states[i].x)  
        y.append(result.states[i].y) 
    plt.plot(y, x, color='green', linewidth=2.0)  
    plt.show()

if __name__ == '__main__':
    main()
