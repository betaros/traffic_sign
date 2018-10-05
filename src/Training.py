"""
Authors:        Jan Füsting
Last edited:    10.09.2018
"""
from src.Misc import Misc

import cv2
import os
import pandas as pd
import tarfile
import urllib.request
import zipfile


class Training:
    """
    Training class
    """
    def __init__(self):
        """
        Initialising
        """
        self.misc = Misc()
        self.training_path = os.path.join(self.misc.project_root, "dataset", "GTSRB", "Final_Training", "Images")
        self.test_path = os.path.join(self.misc.project_root, "dataset", "GTSRB", "Final_Test", "Images")
    
    def download_pos_files(self, images=True, haar=False):
        """
        Downloads and extracts the datasets selected
        :param images:
        :param haar:
        :return:
        """
        url = "http://benchmark.ini.rub.de/Dataset/"

        file_list = ["GTSRB_Final_Training_Images.zip",
                     "GTSRB_Final_Test_Images.zip",
                     "GTSRB_Final_Training_Haar.zip",
                     "GTSRB_Final_Test_Haar.zip"]

        for file in file_list:
            name_split = file.split("_")
            name_split[3] = name_split[3][:-4]
            folder_path = os.path.join(self.misc.project_root, "dataset",
                                       name_split[0],
                                       "_".join(name_split[1:3]),
                                       name_split[3])

            haar2 = haar and name_split[3] == "Haar"
            images2 = images and name_split[3] == "Images"

            if os.path.exists(folder_path) and (haar2 or images2):
                self.misc.logger.debug("Skip downloading %s", file)
                continue

            link = url + file
            store = os.path.join(self.misc.project_root, file)
            extract_path = os.path.join(self.misc.project_root, "dataset")

            if not os.path.isfile(store):
                try:
                    self.misc.logger.debug("Downloading files to %s", store)
                    urllib.request.urlretrieve(link, store)
                    self.misc.logger.debug("Finished downloading")
                except (urllib.request.HTTPError, urllib.request.URLError):
                    self.misc.logger.error("Unable to download data")

                try:
                    zf = zipfile.ZipFile(store)
                    self.misc.logger.debug("Extracting %s", file)
                    zf.extractall(path=extract_path)
                    self.misc.logger.debug("Extraction complete")
                    zf.close()

                    os.remove(store)
                except (ValueError, RuntimeError):
                    self.misc.logger.error("Unable to extract data")
            else:
                self.misc.logger.debug("Skip downloading %s", file)

    def download_neg_files(self):
        """
        Downloading negative sample files

        Source: http://www.robots.ox.ac.uk/~vgg/data/bicos/
        """
        url = "http://www.robots.ox.ac.uk/~vgg/data/bicos/data/"
        file = "airplanes.tar"

        link = os.path.join(url, file)
        store = os.path.join(self.misc.project_root, file)
        extract_path = os.path.join(self.misc.project_root, "dataset")

        if os.path.exists(os.path.join(extract_path, "airplanes")):
            self.misc.logger.debug("Skip downloading %s", file)
            return

        if not os.path.isfile(store):
            try:
                self.misc.logger.debug("Downloading files to %s", store)
                urllib.request.urlretrieve(link, store)
                self.misc.logger.debug("Finished downloading")
            except (urllib.request.HTTPError, urllib.request.URLError):
                self.misc.logger.error("Unable to download data")

            try:
                self.misc.logger.debug("Extracting %s", file)
                with tarfile.open(store) as tar:
                    subdir_and_files = [
                        tarinfo for tarinfo in tar.getmembers()
                        if tarinfo.name.startswith("airplanes/jpg/")
                    ]
                    tar.extractall(members=subdir_and_files, path=extract_path)
                self.misc.logger.debug("Extraction complete")
                os.remove(store)
            except (ValueError, RuntimeError):
                self.misc.logger.error("Unable to extract data")
        else:
            self.misc.logger.debug("Skip downloading %s", file)

    def download_face_recognition_haar(self):
        """
        Downloading face recognition haar data as sample data
        :return:
        """
        link = "https://raw.githubusercontent.com/opencv/opencv/master/data/haarcascades/haarcascade_frontalface_default.xml"
        store = os.path.join(self.misc.project_root, "dataset", "haarcascade_frontalface_default.xml")

        if not os.path.isfile(store):
            try:
                self.misc.logger.debug("Downloading files to %s", store)
                urllib.request.urlretrieve(link, store)
                self.misc.logger.debug("Finished downloading")
            except (urllib.request.HTTPError, urllib.request.URLError):
                self.misc.logger.error("Unable to download data")
        else:
            self.misc.logger.debug("Skip downloading haarcascade_frontalface_default.xml")

    def manipulate_image(self):
        """
        Modifies images to gray scale and resize it
        :return:
        """
        gtsrb_exists = False
        neg_exists = False
        traffic_exists = False

        for (dir_path, dir_names, file_names) in os.walk(os.path.join(self.misc.project_root, "dataset")):
            for dir_name in dir_names:
                if dir_name == "Images":
                    image_mod_path = os.path.join(dir_path, "Images_mod")
                    if not os.path.exists(image_mod_path):
                        os.mkdir(image_mod_path)
                        self.misc.logger.debug("Created folder %s", image_mod_path)
                    else:
                        gtsrb_exists = True
                        self.misc.logger.debug("Skip manipulating positive images")
                if dir_name == "jpg":
                    jpg_mod_path = os.path.join(dir_path, "jpg_mod")
                    if not os.path.exists(jpg_mod_path):
                        os.mkdir(jpg_mod_path)
                        self.misc.logger.debug("Created folder %s", jpg_mod_path)
                    else:
                        neg_exists = True
                        self.misc.logger.debug("Skip manipulating negative images")
                if dir_name == "traffic":
                    traffic_mod_path = os.path.join(dir_path, "traffic_mod")
                    if not os.path.exists(traffic_mod_path):
                        os.mkdir(traffic_mod_path)
                        self.misc.logger.debug("Created folder %s", traffic_mod_path)
                    else:
                        traffic_exists = True
                        self.misc.logger.debug("Skip manipulating positive images")

            for file_name in file_names:
                if file_name.endswith(".ppm") and "Image" in dir_path:
                    if gtsrb_exists:
                        continue

                    img = cv2.imread(os.path.join(dir_path, file_name), cv2.IMREAD_GRAYSCALE)
                    dim = 50
                    resized_image = cv2.resize(img, (dim, dim))
                    resized_image = cv2.cvtColor(resized_image, cv2.COLOR_GRAY2BGR)

                    try:
                        dir_path_mod = os.path.join(dir_path[:-16], "Images_mod", dir_path[-5:])
                        if not os.path.exists(dir_path_mod):
                            os.mkdir(dir_path_mod)
                            self.misc.logger.debug("Created folder %s", dir_path_mod)
                    except ValueError:
                        dir_path_mod = os.path.join(dir_path[:-16], "Images_mod")
                    store_path = os.path.join(dir_path_mod, file_name)
                    cv2.imwrite(store_path, resized_image)
                if file_name.endswith(".jpg") and "airplanes" in dir_path:
                    if neg_exists:
                        continue

                    img = cv2.imread(os.path.join(dir_path, file_name), cv2.IMREAD_GRAYSCALE)
                    dim = 100
                    resized_image = cv2.resize(img, (dim, dim))
                    resized_image = cv2.cvtColor(resized_image, cv2.COLOR_GRAY2BGR)
                    dir_path_mod = os.path.join(dir_path[:-3], "jpg_mod")

                    store_path = os.path.join(dir_path_mod, file_name)
                    cv2.imwrite(store_path, resized_image)
                if file_name.endswith(".jpg") and "traffic" in dir_path:
                    if traffic_exists:
                        continue

                    img = cv2.imread(os.path.join(dir_path, file_name), cv2.IMREAD_GRAYSCALE)
                    dim = 50
                    resized_image = cv2.resize(img, (dim, dim))
                    resized_image = cv2.cvtColor(resized_image, cv2.COLOR_GRAY2BGR)

                    try:
                        dir_path_mod = os.path.join(dir_path[:-10], "traffic_mod", dir_path[-2:])
                        if not os.path.exists(dir_path_mod):
                            os.mkdir(dir_path_mod)
                            self.misc.logger.debug("Created folder %s", dir_path_mod)
                    except ValueError:
                        dir_path_mod = os.path.join(dir_path[:-10], "traffic_mod")
                    store_path = os.path.join(dir_path_mod, file_name)
                    cv2.imwrite(store_path, resized_image)

    def generate_description_gtsrb(self):
        """

        :return:
        """
        class_id = 0

        training_properties = self.load_training_image_properties()
        info_file_path = os.path.join(self.misc.project_root, "info.dat")
        if os.path.isfile(info_file_path):
            os.remove(info_file_path)

        for element in training_properties:
            for index, row in element.iterrows():
                if row['ClassId'] > class_id:
                    class_id = row['ClassId']
                line = os.path.join(self.misc.project_root, "dataset", "GTSRB", "Final_Training", "Images_mod",
                                    self.misc.fill_number(row['ClassId'], 5), row['Filename']) \
                                    + ' 1 ' + str(self.misc.interpolate(row['Roi.X1'], 0, row['Width'], 0, 50)) \
                                    + ' ' + str(self.misc.interpolate(row['Roi.Y1'], 0, row['Height'], 0, 50)) \
                                    + ' ' + str(self.misc.interpolate(row['Roi.X2'] - row['Roi.X1'], 0, row['Width'], 0, 50)) \
                                    + ' ' + str(self.misc.interpolate(row['Roi.Y2'] - row['Roi.Y1'], 0, row['Height'], 0, 50)) + '\n'
                with open(info_file_path, 'a') as f:
                    f.write(line)

                # line = os.path.join(self.misc.project_root, "dataset", "GTSRB", "Final_Training", row['Filename'])
                # with open('bg.txt', 'a') as f:
                #    f.write(line)

    def generate_description_traffic(self):
        """

        :return:
        """
        self.misc.logger.debug("Generate description for traffic dataset")
        root_dir = os.path.join(self.misc.project_root, "dataset", "traffic_mod")
        for dir_name, subdir_list, file_list in os.walk(root_dir):
            if subdir_list:
                for element in subdir_list:
                    info_path = os.path.join(root_dir, element, "info.dat")
                    if os.path.isfile(info_path):
                        self.misc.logger.debug("info.dat already exists")
                        return
            for fname in file_list:
                line = os.path.join(root_dir, dir_name, fname) + " 1 0 0 50 50"
                # self.misc.logger.debug(line)

                with open(os.path.join(root_dir, dir_name, "info.dat"), 'a') as f:
                    line = line + "\n"
                    f.write(line)

    def generate_description_airplanes(self):
        """

        :return:
        """
        self.misc.logger.debug("Generate description for airplanes dataset")
        root_dir = os.path.join(self.misc.project_root, "dataset", "airplanes", "jpg_mod")
        for dir_name, subdir_list, file_list in os.walk(root_dir):
            info_path = os.path.join(root_dir, "bg.txt")
            if os.path.isfile(info_path):
                self.misc.logger.debug("info.dat already exists")
                return
            for fname in file_list:
                line = os.path.join(root_dir, fname)
                # self.misc.logger.debug(line)

                with open(os.path.join(root_dir, dir_name, "bg.txt"), 'a') as f:
                    line = line + "\n"
                    f.write(line)

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
