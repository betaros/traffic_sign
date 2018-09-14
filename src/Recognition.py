"""
Authors:        Jan Füsting
Last edited:    10.09.2018
"""
import cv2
import logging

from Misc import Misc


class Recognition:
    def __init__(self):
        logging.basicConfig(format='%(asctime)s - %(levelname)s - %(funcName)s: %(message)s', level=logging.DEBUG)
        self.logger = logging.getLogger("traffic_sign")

        self.misc = Misc()

    def detect(self):
        self.logger.debug("")

