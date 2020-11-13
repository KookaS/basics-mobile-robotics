from typing import List, Tuple

from src.thymio.Thymio import Thymio


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


def _get_movements_4n() -> List[Tuple[int, int, float]]:
    """
    Get all possible 4-connectivity movements (up, down, left right).
    :return: list of movements with cost [(dx, dy, movement_cost)]
    """
    return [(1, 0, 1.0),
            (0, 1, 1.0),
            (-1, 0, 1.0),
            (0, -1, 1.0)]


def _get_movements_8n():
    """
    Get all possible 8-connectivity movements. Equivalent to get_movements_in_radius(1)
    (up, down, left, right and the 4 diagonals).
    :return: list of movements with cost [(dx, dy, movement_cost)]
    """
    s2 = math.sqrt(2)
    return [(1, 0, 1.0),
            (0, 1, 1.0),
            (-1, 0, 1.0),
            (0, -1, 1.0),
            (1, 1, s2),
            (-1, 1, s2),
            (-1, -1, s2),
            (1, -1, s2)]
