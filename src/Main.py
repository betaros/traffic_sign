"""
This project trains an AI to detect german traffic signs and sends the recognized signs to ros

TODO:
- interpolate ROI from CSV to new dimensions
- integrate ROS platform

Authors:        Jan Füsting
Last edited:    10.09.2018
"""
import os

from src.Misc import Misc
from src.Recognition import Recognition
from src.Training import Training


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

        # Getting and manipulating datasets
        # self.training.download_pos_files(images=True, haar=True)
        # self.training.download_neg_files()
        # self.training.download_face_recognition_haar()
        # self.training.manipulate_image()
        # self.training.generate_description_traffic()
        # self.training.generate_description_airplanes()

        # Get camera image and find traffic signs
        # self.recognition.face_recognition()
        self.recognition.get_camera_image()

        self.misc.logger.debug("Program finished")


main = Main()
main.run()
