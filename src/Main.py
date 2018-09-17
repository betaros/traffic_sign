"""
This project trains an AI to detect german traffic signs and sends the recognized signs to ros

TODO:
- interpolate ROI from CSV to new dimensions
- integrate ROS platform

Authors:        Jan Füsting
Last edited:    10.09.2018
"""
import os

from Misc import Misc
from Recognition import Recognition
from Training import Training


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
        self.training = Training()

    def run(self):
        """
        This method controls program sequence
        :return:
        """

        # Initialize system
        self.misc.logger.debug("Program started")
        dataset_path = self.misc.project_root + "/dataset"
        if not os.path.exists(dataset_path):
            os.makedirs(dataset_path)

        # Getting the files
        self.misc.download_pos_files(images=True, haar=True)
        self.misc.manipulate_image(positive=True)

        self.misc.download_face_recognition_haar()

        # self.training.train_system()

        # Get camera image and find traffic signs
        self.recognition.face_recognition()

        self.misc.logger.debug("Program finished")


main = Main()
main.run()
