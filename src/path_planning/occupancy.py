import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors
from src.path_planning.a_star import A_Star

# constants
LENGTH = 29
WIDTH = 32

LOCALIZATION = 0
OCCUPANCY = 1
FREE = 0
OCCUPIED = 1


def display_map(grid, type_map):
    """
    Display a map (either localization grid or occupancy grid)

    :param grid: 2D matrix containing the values of each cell in the map
    :param type_map: specify the type of map  and can take 2 values (LOCALIZATION or OCCUPANCY)

    :return: the fig and ax objects.
    """
    fig, ax = plt.subplots(figsize=(7, 7))

    major_ticks_x = np.arange(0, WIDTH, 5)
    minor_ticks_x = np.arange(0, WIDTH, 1)
    major_ticks_y = np.arange(0, LENGTH, 5)
    minor_ticks_y = np.arange(0, LENGTH, 1)
    ax.set_xticks(major_ticks_x)
    ax.set_xticks(minor_ticks_x, minor=True)
    ax.set_yticks(major_ticks_y)
    ax.set_yticks(minor_ticks_y, minor=True)
    ax.grid(which='minor', alpha=0.2)
    ax.grid(which='major', alpha=0.5)
    ax.set_ylim([0, (LENGTH - 1)])
    ax.set_xlim([0, (WIDTH - 1)])
    ax.grid(True)

    if type_map == OCCUPANCY:
        # Select the colors with which to display obstacles and free cells
        cmap = colors.ListedColormap(['white', 'red'])

        # Displaying the map
        # ax.imshow(grid, cmap=cmap, extent=[0, 42, 0, 45])
        ax.imshow(grid, cmap=cmap)
        plt.title("Map : free cells in white, occupied cells in red")

    elif type_map == LOCALIZATION:
        cmap = colors.ListedColormap(['white', 'black'])

        # Displaying the map
        ax.imshow(grid, cmap=cmap, extent=[0, (WIDTH - 1), 0, (LENGTH - 1)])
        plt.title("Localization grid")

    return fig, ax


def create_occupancy_grid():
    """
    Create the occupancy grid

    return: A 2D matrix filled with 0's (free cells) and 1's (occupied cells) representing the occupancy map
    """
    occupancy_grid = np.zeros((WIDTH, LENGTH))

    # obstacle 1
    # occupancy_grid[14:16, 40:45] = OCCUPIED

    # obstacle 2
    occupancy_grid[0:6, 18] = OCCUPIED
    occupancy_grid[4:5, 19] = OCCUPIED
    occupancy_grid[4:5, 20] = OCCUPIED
    occupancy_grid[4:5, 21] = OCCUPIED
    occupancy_grid[4, 22] = OCCUPIED

    # obstacle 3
    # occupancy_grid[34:, 29] = OCCUPIED

    # obstacle 4
    occupancy_grid[24:27, 11:14] = OCCUPIED

    return occupancy_grid


def create_localization_grid():
    """
    Create the localization grid, that will help Thymio to localize itself

    return: A 2D matrix filled with 0's (white cells) and 1's (black cells) representing the localization map
    """
    localization_grid = np.zeros((LENGTH, WIDTH))

    localization_grid[0, :] = [1, 0, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 0, 1, 0, 1, 0, 1, 1, 0, 1, 1]
    # 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0]
    localization_grid[1, :] = [0, 1, 0, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
    # 1, 1, 0, 1, 1, 1, 1, 1, 0, 0, 1, 1]
    localization_grid[2, :] = [0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0]
    # 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0]
    localization_grid[3, :] = [1, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 1, 0, 1, 0, 1, 0, 1, 1, 0, 0, 1, 0, 1, 0, 0, 1, 1]
    # 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1]
    localization_grid[4, :] = [1, 1, 1, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1]
    # 1, 1, 1, 0, 0, 0, 1, 1, 0, 1, 1, 1]
    localization_grid[5, :] = [1, 0, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 0, 1, 1, 0]
    # 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0]
    localization_grid[6, :] = [0, 1, 0, 1, 1, 1, 0, 0, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0]
    # 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0]
    localization_grid[7, :] = [0, 1, 0, 1, 1, 1, 0, 0, 0, 0, 1, 1, 0, 1, 0, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0]
    # 1, 1, 0, 0, 0, 1, 1, 0, 0, 1, 1, 1]
    localization_grid[8, :] = [0, 1, 1, 0, 1, 0, 0, 1, 1, 1, 1, 0, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 0, 0]
    # 0, 1, 1, 0, 1, 0, 0, 0, 1, 1, 0, 0]
    localization_grid[9, :] = [0, 1, 0, 0, 1, 1, 1, 1, 0, 1, 0, 1, 1, 0, 0, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 0, 1, 0, 1]
    # 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1]
    localization_grid[10, :] = [1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0]
    # 0, 1, 1, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0]
    localization_grid[11, :] = [0, 1, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 1, 0, 1, 0, 0, 1, 1, 1, 1, 0]
    # 1, 0, 0, 1, 1, 1, 0, 1, 1, 0, 0, 1, 0]
    localization_grid[12, :] = [1, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 0, 0, 1, 0, 1, 0, 0, 1]
    # 0, 1, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1, 0]
    localization_grid[13, :] = [0, 0, 0, 1, 1, 0, 1, 1, 0, 0, 1, 1, 1, 1, 1, 0, 0, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 0, 0]
    # 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 0, 1, 1]
    localization_grid[14, :] = [1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 1, 1]
    # 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0]

    localization_grid[15, :] = [0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1]
    # 1, 1, 0, 0, 1, 1, 1, 0, 1, 1, 0, 1, 1]
    localization_grid[16, :] = [1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 1, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1]
    # 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 1]
    localization_grid[17, :] = [1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1]
    # 0, 0, 0, 0, 1, 1, 0, 1, 1, 1, 0, 1, 0]
    localization_grid[18, :] = [0, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1]
    # 0, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 1]
    localization_grid[19, :] = [0, 0, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1, 0]
    # 1, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 1, 1]
    localization_grid[20, :] = [1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0]
    # 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0]
    localization_grid[21, :] = [0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 0, 0, 0, 1, 0, 1, 1]
    # 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 1]
    localization_grid[22, :] = [1, 1, 0, 1, 0, 0, 1, 1, 1, 0, 1, 1, 0, 1, 0, 1, 0, 1, 0, 0, 1, 1, 0, 1, 1, 1, 1, 0, 1]
    # 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 1, 1, 0]
    localization_grid[23, :] = [0, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 1, 0, 0]
    # 1, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1, 1, 0]
    localization_grid[24, :] = [1, 1, 0, 1, 0, 0, 0, 1, 0, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 0, 1]
    # 0, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1]
    localization_grid[25, :] = [0, 0, 0, 1, 0, 1, 1, 0, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1]
    # 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 0, 0]
    localization_grid[26, :] = [0, 0, 0, 1, 1, 0, 1, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0]
    # 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 1, 0]
    localization_grid[27, :] = [1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 0, 1, 0, 1, 1, 0, 1, 1, 0, 0, 0, 1]
    # 0, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1]
    localization_grid[28, :] = [0, 1, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 0, 0, 1, 1, 1, 0, 1, 1, 1, 0, 0, 1, 1, 0, 0, 1, 0]
    # 1, 0, 1, 1, 0, 1, 1, 0, 0, 1, 1, 1, 0]
    localization_grid[29, :] = [0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 0, 0]
    # 0, 1, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1, 0]
    localization_grid[30, :] = [1, 0, 0, 0, 1, 1, 0, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 1]
    # 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 0, 0, 0]

    localization_grid[31, :] = [0, 0, 0, 0, 0, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0, 1, 1, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0]
    # 1, 0, 0, 0, 1, 1, 0, 1, 0, 1, 1, 1, 1]
    '''
    localization_grid[32, :] = [0, 0, 1, 1, 1, 0, 0, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1, 1, 0,
                                0, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 0]
    localization_grid[33, :] = [1, 1, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 1, 1, 1, 1, 0, 0, 1, 0, 1, 0, 0, 1, 1, 1, 1,
                                1, 0, 1, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0]
    localization_grid[34, :] = [1, 1, 0, 1, 1, 1, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 0, 0, 1, 1, 0,
                                0, 1, 0, 0, 1, 0, 1, 0, 1, 1, 0, 1, 0]
    localization_grid[35, :] = [0, 0, 1, 1, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 0, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1,
                                0, 1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0]
    localization_grid[36, :] = [0, 1, 0, 1, 0, 1, 1, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 1, 0, 0,
                                1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1]
    localization_grid[37, :] = [1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 1, 0, 0, 1, 1, 1, 1, 1,
                                1, 0, 1, 0, 1, 1, 0, 0, 0, 0, 1, 0, 0]
    localization_grid[38, :] = [0, 1, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0,
                                1, 0, 1, 1, 0, 1, 1, 0, 1, 0, 0, 0, 0]
    localization_grid[39, :] = [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 1, 0, 1, 1,
                                1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1]
    localization_grid[40, :] = [0, 1, 0, 0, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 1, 0, 1, 0, 0, 0, 1, 1,
                                0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1]
    localization_grid[41, :] = [1, 0, 1, 1, 1, 0, 0, 1, 0, 0, 1, 1, 0, 0, 0, 0, 1, 0, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1,
                                0, 1, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0]
    localization_grid[42, :] = [0, 1, 0, 0, 1, 1, 1, 1, 1, 0, 1, 0, 0, 1, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1,
                                0, 1, 0, 0, 0, 1, 1, 0, 1, 1, 1, 0, 1]
    localization_grid[43, :] = [1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 0, 1, 0,
                                0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 1, 1, 1]
    localization_grid[44, :] = [0, 1, 1, 0, 1, 0, 1, 0, 1, 1, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 1, 1,
                                1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1]
'''
    return localization_grid


def increased_obstacles_map(occupancy_grid):
    nb_rows = len(occupancy_grid)
    nb_cols = len(occupancy_grid[0])
    increased_occupancy_grid = np.zeros([nb_rows + 6, nb_cols + 6])

    for i in range(len(occupancy_grid)):
        for j in range(len(occupancy_grid[0])):

            if occupancy_grid[i, j] == OCCUPIED:
                increased_occupancy_grid[i:i + 7, j:j + 7] = np.ones([7, 7])

    final_occupancy_grid = increased_occupancy_grid[3:(WIDTH + 3), 3:(LENGTH + 3)]
    return final_occupancy_grid


def display_global_path(start, goal, path, occupancy_grid):
    # Displaying the map
    fig_astar, ax_astar = display_map(occupancy_grid, OCCUPANCY)
    # ax_astar.imshow(occupancy_grid.transpose(), cmap=cmap)

    # Plot the best path found and the list of visited nodes
    ax_astar.plot(path[0], path[1], marker="o", color='blue')
    ax_astar.scatter(start[0], start[1], marker="o", color='green', s=200)
    ax_astar.scatter(goal[0], goal[1], marker="o", color='purple', s=200)
    # ax_astar.set_ylim(ax_astar.get_ylim()[::-1])

    ax_astar.set_ylabel('y axis')
    ax_astar.set_xlabel('x axis')
    plt.figure()
    plt.show()


def create_grid():
    occupancy_grid = create_occupancy_grid()
    # display_map(occupancy_grid.transpose(), OCCUPANCY)

    # localization_grid = create_localization_grid()
    # display_map(localization_grid, LOCALIZATION)

    final_occupancy_grid = increased_obstacles_map(occupancy_grid)
    # display_map(occupancy_grid.transpose(), OCCUPANCY)
    # display_map(final_occupancy_grid.transpose(), OCCUPANCY)
    return final_occupancy_grid


def path_to_command_thymio(path):
    RIGHT = 0
    LEFT = 1
    STRAIGHT = 2

    current_x = path[0][0]
    current_y = path[1][0]

    next_x = path[0][1]
    next_y = path[1][1]

    # next-prev
    delta_x = path[0][1] - path[0][0]
    delta_y = path[1][1] - path[1][0]

    # delat_x = 0 and delta_y = -/+ 1 (or delat_x = -/+ 1 and delta_y = 0): go straight
    turn = STRAIGHT

    # delat_x = -1 and delta_y = 1 (or delat_x = 1 and delta_y = -1): turn to the right
    if delta_x * delta_y < 0:
        turn = RIGHT

    # delat_x = -1 and delta_y = -1 (or delat_x = 1 and delta_y = 1): turn to the left
    if delta_x * delta_y == 1:
        turn = LEFT

    new_path = np.array([path[0][1:], path[1][1:]])

    return turn, new_path


def full_path_to_points(path):
    points_x = [path[0][0]]
    points_y = [path[1][0]]

    new_path = path
    prev_turn, new_path = path_to_command_thymio(new_path)

    for i in range(len(new_path[0]) - 1):

        new_turn, new_path = path_to_command_thymio(new_path)

        if new_turn != prev_turn:
            points_x.append(path[0][i + 1])
            points_y.append(path[1][i + 1])

        prev_turn = new_turn

    points_x.append(path[0][-1])
    points_y.append(path[1][-1])
    points = [points_x, points_y]

    return points


def display_occupancy(final_occupancy_grid, position, goal):
    # Run the A* algorithm
    x = int(position[1] / 2.5)
    y = LENGTH - int(position[0] / 2.5)
    new_pos = (x, y)
    print("start: ", new_pos)
    print("goal: ", goal)
    print("Welcome to Astar")
    path = A_Star(new_pos, goal, final_occupancy_grid)  # all steps in path
    path = np.array(path).reshape(-1, 2).transpose()
    new_path = full_path_to_points(path)  # concatenated path
    print("path", path)
    print("new_path", new_path)
    display_global_path(new_pos, goal, new_path, final_occupancy_grid.transpose())
    new_path = np.delete(new_path, 0, 1)
    return new_path
