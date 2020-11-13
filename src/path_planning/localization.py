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
