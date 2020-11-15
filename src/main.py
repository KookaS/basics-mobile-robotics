import os
import sys
import pprint
import time

from src.displacement.management import EventHandler
from src.thymio.Thymio import Thymio
from dotenv import load_dotenv

# Adding the src folder in the current directory as it contains the script with the Thymio class
sys.path.insert(0, os.path.join(os.getcwd(), 'src'))
load_dotenv()


def print_thymio(thymio: Thymio):
    """
    Print the variables of Thymio

    :param thymio: The file location of the spreadsheet
    """

    print('All Thymio instance attributes:')
    pprint.pprint(dir(thymio))

    # see what the different read-write variables that you can access are
    variables = thymio.variable_description()
    print('\nVariables of Thymio:')
    for var in variables:
        print(var)


def main():
    th = Thymio.serial(port=os.getenv("COM_PORT"), refreshing_rate=0.1)
    time.sleep(3)  # To make sure the Thymio has had time to connect
    # print_thymio(th)

    # rotate(thymio=th, angle=90.0, verbose=True)
    # state = State(th)
    # state.acquire_data()
    handler = EventHandler(th, interval_check=5)  # check every interval_check seconds to change scenarios
    print("end of main!")


if __name__ == "__main__":
    main()
