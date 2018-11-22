"""
Authors:        Jan Füsting
Last edited:    17.09.2018
"""
import logging
import numpy as np
import os


class Misc:
    """
    This class contains functions which are used independent of it's context
    """
    def __init__(self):
        """
        Initialising
        """
        logging.basicConfig(format='%(asctime)s - %(levelname)s - %(filename)s - %(funcName)s - %(message)s',
                            level=logging.DEBUG)
        self.logger = logging.getLogger("traffic_sign")
        self.project_root = os.path.dirname(os.path.dirname(__file__))


    @staticmethod
    def fill_number(number, length):
        """
        Fills string with leading zeros 1 -> 00001
        :param number:
        :param length:
        :return:
        """
        return ("0" * (length - len(str(number)))) + str(number)

    @staticmethod
    def folder_counter(path):
        """
        Counts the number of folders inside a folder
        :param path:
        :return:
        """
        folders = 0
        for _, dirnames, _ in os.walk(path):
            folders += len(dirnames)
        return folders

    @staticmethod
    def interpolate(value, old_start, old_end, new_start, new_end):
        """
        Interpolates values to new scale

        :param value:
        :param old_start:
        :param old_end:
        :param new_start:
        :param new_end:
        :return: integer
        """
        delta_old = old_end - old_start
        delta_new = new_end - new_start

        value = value / delta_old
        value = value * delta_new
        value = value + new_start

        return int(value)
