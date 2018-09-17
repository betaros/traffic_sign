"""
Authors:        Jan Füsting
Last edited:    10.09.2018
"""
from Misc import Misc

import cv2
import logging
from math import cos, sin
import matplotlib.pyplot as plt
import os
import pandas as pd
from PIL import Image


class Training:
    """
    Training class
    """
    def __init__(self):
        """
        Initialising
        """
        self.training_path = os.path.join(self.misc.project_root, "dataset", "GTSRB", "Final_Training", "Images")
        self.test_path = os.path.join(self.misc.project_root, "dataset", "GTSRB", "Final_Test", "Images")

        self.misc = Misc()

    def load_training_image_properties(self):
        """
        Loads training images properties
        :return:
        """
        iterator_training = Misc.folder_counter(self.training_path)
        image_list = []

        for i in range(0, iterator_training):
            class_number = Misc.fill_number(i, 5)
            training_path_class = os.path.join(self.training_path, class_number)
            csv_data = None
            for file in os.listdir(training_path_class):
                if file.endswith(".csv"):
                    csv_path = os.path.join(training_path_class, file)
                    csv_data = pd.read_csv(filepath_or_buffer=csv_path, sep=';')
                    # self.misc.logger.debug("CSV path: %s", csv_path)
                    break
            image_list.append(csv_data)
        return image_list

    def load_test_image_properties(self):
        """
        Loads test images properties
        :return:
        """
        image_list = []
        csv_data = None
        for file in os.listdir(self.test_path):
            if file.endswith(".csv"):
                csv_path = os.path.join(self.test_path, file)
                csv_data = pd.read_csv(csv_path)
                # self.misc.logger.debug("CSV path: %s", csv_path)
                break
        image_list.append(csv_data)
        return image_list

    def load_images(self, training):
        """
        Load images from datasets
        :param training:
        :return:
        """
        if training:
            self.misc.logger.debug("Training set")
            image_list = self.load_training_image_properties()
        else:
            self.misc.logger.debug("Test set")
            image_list = self.load_training_image_properties()

        for element in image_list:
            self.misc.logger.debug("Class id: %i", element["ClassId"][0])
            for index, row in element.iterrows():
                img_path = os.path.join(self.training_path, Misc().fill_number(row['ClassId'], 5), row['Filename'])
                # if index == 0:
                    # self.show(img_path, row['Roi.X1'], row['Roi.Y1'], row['Roi.X2'], row['Roi.Y2'])
                self.misc.logger.debug(str(row['ClassId']) + " " +
                                       row['Filename'] + " " +
                                       str(row['Height']) + " " +
                                       str(row['Width']) + " " +
                                       str(row['Roi.X1']) + " " +
                                       str(row['Roi.Y1']) + " " +
                                       str(row['Roi.X2']) + " " +
                                       str(row['Roi.Y2']))
        return None

    def train_system(self):
        """
        Trains the ai
        :return:
        """
        self.load_images(training=True)

    def show(self, image, x1=0, y1=0, x2=0, y2=0):
        """

        :param image:
        :param x1:
        :param y1:
        :param x2:
        :param y2:
        :return:
        """
        # Image.open(image).show()
        img = cv2.imread(image)
        img_selected = cv2.rectangle(img,
                                 (x1, y1),
                                 (x2, y2),
                                 (0, 255, 0), 2)
        cv2.imshow("Image", img_selected)
        cv2.waitKey(0)

