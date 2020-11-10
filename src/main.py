import os
import sys
import pprint
from src.thymio.Thymio import Thymio
from dotenv import load_dotenv

load_dotenv()

# Adding the src folder in the current directory as it contains the script
# with the Thymio class
sys.path.insert(0, os.path.join(os.getcwd(), 'src'))


def main():
    th = Thymio.serial(port=os.getenv("COM_PORT"), refreshing_rate=0.1)  # ser:device=\\.\COM4
    print_thymio(th)


def print_thymio(th):
    print('All Thymio instance attributes:')
    pprint.pprint(dir(th))

    # see what the different read-write variables that you can access are
    variables = th.variable_description()
    print('\nVariables of Thymio:')
    for var in variables:
        print(var)


if __name__ == "__main__":
    main()
