"""
This project trains an AI to detect german traffic signs and sends the recognized signs to ros

TODO:
- integrate ROS platform

Authors:        Jan Füsting
Last edited:    10.09.2018
"""
# Fixes error Kinetic and OpenCV
import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')

from src.Misc import Misc
from src.Recognition import Recognition


# Conflict ROS Kinetic and OpenCV
# https://stackoverflow.com/questions/43019951/after-install-ros-kinetic-cannot-import-opencv
class Main:
    """
    Main class
    """
    def __init__(self):
        """
        Initialization
        """
        self.misc = Misc()
        self.recognition = Recognition()

    def run(self):
        """
        This method controls program sequence
        :return:
        """

        # Initialize system
        self.misc.logger.debug("Program started")

        # Get camera image and find traffic signs
        self.recognition.get_camera_image()

        self.misc.logger.debug("Program finished")


main = Main()
main.run()
