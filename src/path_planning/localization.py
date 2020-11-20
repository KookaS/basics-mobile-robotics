from src.path_planning.a_star import A_Star
from src.thymio.Thymio import Thymio
import cv2
import matplotlib.pyplot as plt
import numpy as np
import math
from matplotlib import colors


def test_ground_white(thymio: Thymio, white_threshold: int, verbose: bool = False):
    """
    Tests whether the two ground sensors have seen white

    :param thymio:          The file location of the spreadsheet
    :param white_threshold: threshold starting which it is considered that the ground sensor saw white
    :param verbose:         whether to print status messages or not
    """
    if all([x > white_threshold for x in thymio['prox.ground.reflected']]):
        if verbose: print("\t\t Saw white on the ground")
        return True
    return False


def test_saw_black(thymio: Thymio, white_threshold: int, verbose: bool = True):
    """
    Line following behaviour of the FSM

    :param thymio:          The file location of the spreadsheet
    :param white_threshold: threshold starting which it is considered that the ground sensor saw white
    :param verbose:         whether to print status messages or not
    """

    if any([x <= white_threshold for x in thymio['prox.ground.reflected']]):
        if verbose: print("\t\t Both ground sensors saw black")
        return True
    return False


class Localization:

    def __init__(self):
        # init value for value setting
        self.color_threshold = 150
        self.zero_init = 0
        self.b, self.g, self.r = (0, 1, 2)
        self.Border = 0
        self.goal, self.thymio = (0, 1)
        self.x, self.y = (0, 1)

        # constants
        self.LOCALIZATION = 0
        self.OCCUPANCY = 1
        self.FREE = 0
        self.OCCUPIED = 1

        #####################################
        # dans prog tif passer de 45 a 44 et 42 a 41?
        #####################################
        # shapren parameter
        self.alpha = 1.5  # Contrast control (1.0-3.0)
        self.beta = 0  # Brightness control (0-100)

    def detect_and_rotate(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # hsv value for the various mask
        lower_blue = np.array([110, 50, 50])
        upper_blue = np.array([132, 255, 255])
        low_green = np.array([36, 0, 0])
        up_green = np.array([86, 255, 255])
        low_red = np.array([178, 179, 0])
        up_red = np.array([[255, 255, 255]])

        # computing of the blue mask to isolate the contours of the map
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
        # find the outside blue contours of the map on the whole world
        contours, _ = cv2.findContours(mask_blue, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)

        # find the rectangle which includes the contours
        max_area = 0
        best = None
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > max_area:
                max_area = area
                best = contour

        rect = cv2.minAreaRect(best)
        box = cv2.boxPoints(rect)
        box = np.int0(box)

        # crop image inside bounding box
        scale = 1
        W = rect[1][0]
        H = rect[1][1]

        # finding the box to rotate
        Xs = [i[0] for i in box]
        Ys = [i[1] for i in box]
        x1 = min(Xs)
        x2 = max(Xs)
        y1 = min(Ys)
        y2 = max(Ys)

        # finding the rotation angle
        angle = rect[2]
        rotated = False
        if angle < -45:
            angle += 90
            rotated = True

        # rotation center and rotation matrix
        center = (int((x1 + x2) / 2), int((y1 + y2) / 2))
        size = (int(scale * (x2 - x1)), int(scale * (y2 - y1)))
        M = cv2.getRotationMatrix2D((size[0] / 2, size[1] / 2), angle, 1.0)

        # cropping the image and rotating it
        cropped = cv2.getRectSubPix(image, size, center)
        cropped = cv2.warpAffine(cropped, M, size)
        croppedW = W if not rotated else H
        croppedH = H if not rotated else W

        corrected = cv2.getRectSubPix(cropped, (int(croppedW * scale), int(croppedH * scale)),
                                      (size[0] / 2, size[1] / 2))
        final_grid = np.array(corrected)
        return final_grid

    def resize(self, final_grid, alpha, beta):
        adjusted = cv2.convertScaleAbs(final_grid, alpha, beta)
        sharpen_kernel = np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]])
        sharpen = cv2.filter2D(final_grid, -1, sharpen_kernel)

        # resize the map to the wanted size
        map_w_border_row = 47
        map_w_border_col = 44
        vis_map = cv2.resize(sharpen, (map_w_border_col, map_w_border_row), cv2.INTER_AREA)
        return vis_map

    def rotate(self, vis_map):
        low_green = np.array([36, 0, 0])
        up_green = np.array([86, 255, 255])
        # computing of the green mask to find the correct orientation of the map
        hsv = cv2.cvtColor(vis_map, cv2.COLOR_BGR2HSV)
        mask_green = cv2.inRange(hsv, low_green, up_green)
        grid_corner = np.array(mask_green)
        n_rows, n_cols = grid_corner.shape

        # turning the whole map until the orientation is good or let it in the initial position
        test_full_rot = 0
        rotation_marker_row = 0
        rotation_marker_col = n_cols - 1

        while grid_corner[rotation_marker_row][n_cols - 1] == 0:
            grid_corner = np.rot90(grid_corner)
            vis_map = np.rot90(vis_map)
            n_rows, n_cols = grid_corner.shape
            test_full_rot = test_full_rot + 1
            if test_full_rot == 4:
                break

        # final world map with goal, thymio and obstacle
        world = vis_map[1:n_rows - 1, 1:n_cols - 1]
        return world

    def detect_object(self, world):
        lower_blue = np.array([110, 50, 50])
        upper_blue = np.array([132, 255, 255])

        # create the map with only the obstucale to non-zero
        world4hsv = world[:, :, ::-1]
        world_hsv = cv2.cvtColor(world4hsv, cv2.COLOR_BGR2HSV)
        mask_red = cv2.inRange(world_hsv, lower_blue, upper_blue)
        occupancy_grid = np.array(mask_red)
        world_rows, world_cols, _ = world.shape
        obstacle_grid = [[[self.zero_init, self.zero_init, self.zero_init] for r in range(world_cols)] for c in
                         range(world_rows)]
        thymio_x, thymio_y = (self.zero_init, self.zero_init)

        # look for the obstacle and increase there size
        for i in range(world_rows):
            for j in range(world_cols):
                occupancy_grid[i][j] = int(occupancy_grid[i][j] / 255)
                # find the thymio coordinate on the world map
                if (world[i][j][self.g] > self.color_treshold) and (world[i][j][self.b] < self.color_treshold):
                    thymio_x, thymio_y = (i, j)
                # find the goal coordinate on the world map
                elif (world[i][j][self.b] > self.color_treshold) and (world[i][j][self.g] < self.color_treshold):
                    goal_x, goal_y = (i, j)
                else:
                    continue
        object_grid = [[goal_x, goal_y], [thymio_x, thymio_y]]
        return object_grid, occupancy_grid

    def vision(self, image):
        final_grid = self.detect_and_rotate(image)
        vis_map = self.resize(final_grid, self.alpha, self.beta)
        world = self.rotate(vis_map)
        object_grid, occupancy_grid = self.detect_object(world)

        return object_grid, occupancy_grid, world

    def display_global_path(self, start, goal, path, occupancy_grid):
        # Displaying the map
        fig_astar, ax_astar = self.display_map(occupancy_grid, self.OCCUPANCY)
        # ax_astar.imshow(occupancy_grid.transpose(), cmap=cmap)

        # Plot the best path found and the list of visited nodes
        ax_astar.plot(path[0], path[1], marker="o", color='orange');
        ax_astar.scatter(start[0], start[1], marker="o", color='green', s=200);
        ax_astar.scatter(goal[0], goal[1], marker="o", color='purple', s=200);
        ax_astar.set_ylim(ax_astar.get_ylim()[::-1])

    def increased_obstacles_map(self, occupancy_grid):
        nb_rows, nb_cols = occupancy_grid.shape
        increased_occupancy_grid = np.zeros([nb_rows + 6, nb_cols + 6])
        for i in range(nb_rows):
            for j in range(nb_cols):

                if occupancy_grid[i, j] == self.OCCUPIED:
                    increased_occupancy_grid[i:i + 7, j:j + 7] = np.ones([7, 7])

        final_occupancy_grid = increased_occupancy_grid[3:44, 3:47]
        return final_occupancy_grid

    def display_map(self, grid, type_map):
        """
        Display a map (either localization grid or occupancy grid)

        :param grid: 2D matrix containing the values of each cell in the map
        :param type_map: specify the type of map  and can take 2 values (LOCALIZATION or OCCUPANCY)
        """

        fig, ax = plt.subplots(figsize=(7, 7))

        major_ticks_x = np.arange(0, 41 + 1, 5)
        minor_ticks_x = np.arange(0, 41 + 1, 1)
        major_ticks_y = np.arange(0, 44 + 1, 5)
        minor_ticks_y = np.arange(0, 44 + 1, 1)
        ax.set_xticks(major_ticks_x)
        ax.set_xticks(minor_ticks_x, minor=True)
        ax.set_yticks(major_ticks_y)
        ax.set_yticks(minor_ticks_y, minor=True)
        ax.grid(which='minor', alpha=0.2)
        ax.grid(which='major', alpha=0.5)
        ax.set_ylim([0, 44])
        ax.set_xlim([0, 41])
        ax.grid(True)

        if type_map == self.OCCUPANCY:
            # Select the colors with which to display obstacles and free cells
            cmap = colors.ListedColormap(['white', 'red'])

            # Displaying the map
            # ax.imshow(grid, cmap=cmap, extent=[0, 42, 0, 45])
            ax.imshow(grid, cmap=cmap)
            plt.title("Map : free cells in white, occupied cells in red");

        elif type_map == self.LOCALIZATION:
            cmap = colors.ListedColormap(['white', 'black'])

            # Displaying the map
            ax.imshow(grid, cmap=cmap, extent=[0, 42, 0, 44])
            plt.title("Localization grid");

        return fig, ax

    def localize(self):
        # open image images/mapf.png
        image = cv2.imread('images/mapf.png')

        object_grid, occupancy_grid, world = self.vision(image)

        # change to the right coordinate format
        occupancy_grid = (np.flipud(occupancy_grid)).transpose()
        # display_map(occupancy_grid.transpose(), OCCUPANCY)

        # thymio and goal coordinate
        thymio_x = object_grid[self.thymio][self.y]
        thymio_y = 44 - object_grid[self.thymio][self.x]
        goal_x = object_grid[self.goal][self.y]
        goal_y = 44 - object_grid[self.goal][self.x]

        start = (thymio_x, thymio_y)
        goal = (goal_x, goal_y)

        final_occupancy_grid = self.increased_obstacles_map(occupancy_grid)

        # Run the A* algorithm
        path = A_Star(start, goal, final_occupancy_grid)
        path = np.array(path).reshape(-1, 2).transpose()

        self.display_global_path(start, goal, path, final_occupancy_grid.transpose())

        # arrange axis for imshow
        fig, ax = plt.subplots(figsize=(7, 7))
        major_ticks_x = np.arange(0, 41 + 1, 5)
        minor_ticks_x = np.arange(0, 41 + 1, 1)
        major_ticks_y = np.arange(0, 44 + 1, 5)
        minor_ticks_y = np.arange(0, 44 + 1, 1)
        ax.set_xticks(major_ticks_x)
        ax.set_xticks(minor_ticks_x, minor=True)
        ax.set_yticks(major_ticks_y)
        ax.set_yticks(minor_ticks_y, minor=True)
        ax.grid(which='minor', alpha=0.2)
        ax.grid(which='major', alpha=0.5)
        ax.set_ylim([0, 44])
        ax.set_xlim([0, 41])
        ax.grid(True)
        plt.imshow(world[:, :, ::-1])
