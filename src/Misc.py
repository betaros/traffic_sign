"""
Authors:        Jan Füsting
Last edited:    10.09.2018
"""
import cv2
import logging
import numpy as np
import os
import urllib.request
import zipfile


class Misc:
    def __init__(self):
        logging.basicConfig(format='%(asctime)s - %(levelname)s - %(filename)s - %(funcName)s - %(message)s',
                            level=logging.DEBUG)
        self.logger = logging.getLogger("traffic_sign")
        self.project_root = os.path.dirname(os.path.dirname(__file__))

        self.sign_classes = ["speed_20",
                             "speed_30",
                             "speed_50",
                             "speed_60",
                             "speed_70",
                             "speed_80",
                             "speed_80_end",
                             "speed_100",
                             "speed_120",
                             "overtake_forbidden_all",
                             "overtake_forbidden_truck",
                             "priority_crossing",
                             "priority_road",
                             "give_way",
                             "stop",
                             "all_vehicle_forbidden",
                             "trucks_forbidden",
                             "entry_forbidden",
                             "attention",
                             "warning_left_turn",
                             "warning_right_turn",
                             "double_turn_left",
                             "bumps",
                             "slippery",
                             "narrow_road_right",
                             "construction_work",
                             "traffic_light",
                             "pedestrians",
                             "children",
                             "bicycle",
                             "snow",
                             "animals",
                             "speed_unlimited",
                             "right_turn",
                             "left_turn",
                             "straight",
                             "straight_or_right_turn",
                             "straight_or_left_turn",
                             "stay_left",
                             "stay_right",
                             "circle_road",
                             "overtake_allowed_all",
                             "overtake_allowed_truck"]
        self.sign_classes_colors = np.random.uniform(0, 255, size=(len(self.sign_classes), 3))

    """
    This class contains miscellaneous functions
    """
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
            folder_path = os.path.join(self.project_root, "dataset",
                                       name_split[0],
                                       "_".join(name_split[1:3]),
                                       name_split[3])

            haar2 = haar and name_split[3] == "Haar"
            images2 = images and name_split[3] == "Images"

            if os.path.exists(folder_path) and (haar2 or images2):
                self.logger.debug("Skip downloading %s", file)
                continue

            link = url + file
            store = os.path.join(self.project_root, file)
            extract_path = os.path.join(self.project_root, "dataset")

            if not os.path.isfile(store):
                try:
                    self.logger.debug("Downloading files to %s", store)
                    urllib.request.urlretrieve(link, store)
                    self.logger.debug("Finished downloading")
                except (urllib.request.HTTPError, urllib.request.URLError):
                    self.logger.error("Unable to download data")

                try:
                    zf = zipfile.ZipFile(store)
                    self.logger.debug("Extracting %s", file)
                    zf.extractall(path=extract_path)
                    self.logger.debug("Extraction complete")
                    zf.close()

                    os.remove(store)
                except (ValueError, RuntimeError):
                    self.logger.error("Unable to extract data")
            else:
                self.logger.debug("Skip downloading %s", file)

    def download_neg_files(self):
        """
        Downloading negative sample files
        :return:
        """
        link = ""

    def download_face_recognition_haar(self):
        link = "https://raw.githubusercontent.com/opencv/opencv/master/data/haarcascades/haarcascade_frontalface_default.xml"
        store = os.path.join(self.project_root, "dataset", "haarcascade_frontalface_default.xml")

        if not os.path.isfile(store):
            try:
                self.logger.debug("Downloading files to %s", store)
                urllib.request.urlretrieve(link, store)
                self.logger.debug("Finished downloading")
            except (urllib.request.HTTPError, urllib.request.URLError):
                self.logger.error("Unable to download data")
        else:
            self.logger.debug("Skip downloading haarcascade_frontalface_default.xml")

    def manipulate_image(self, positive=False):
        """
        Modifies images to gray scale and resize it
        :return:
        """
        for (dir_path, dir_names, file_names) in os.walk(os.path.join(self.project_root, "dataset")):
            for dir_name in dir_names:
                if dir_name == "Images":
                    image_mod_path = os.path.join(dir_path, "Images_mod")
                    if not os.path.exists(image_mod_path):
                        os.mkdir(image_mod_path)
                        self.logger.debug("Created folder %s", image_mod_path)
                    else:
                        return
            for file_name in file_names:
                if file_name.endswith(".ppm"):
                    img = cv2.imread(os.path.join(dir_path, file_name), cv2.IMREAD_GRAYSCALE)
                    if positive:
                        dim = 50
                    else:
                        dim = 100
                    resized_image = cv2.resize(img, (dim, dim))
                    resized_image = cv2.cvtColor(resized_image, cv2.COLOR_GRAY2BGR)
                    try:
                        int(dir_path[-5:])
                        dir_path_mod = os.path.join(dir_path[:-12], "Images_mod", dir_path[-5:])
                        if not os.path.exists(dir_path_mod):
                            os.mkdir(dir_path_mod)
                            self.logger.debug("Created folder %s", dir_path_mod)
                    except ValueError:
                        dir_path_mod = os.path.join(dir_path[:-12], "Images_mod")
                    store_path = os.path.join(dir_path_mod, file_name)
                    cv2.imwrite(store_path, resized_image)

    def generate_description(self):
        """

        :return:
        """
        return 1

    def get_camera_image(self, mirror=True):
        """

        :return:
        """
        self.logger.debug("Show cam")

        face_cascade = cv2.CascadeClassifier(os.path.join(self.project_root, 'dataset', 'haarcascade_frontalface_default.xml'))

        cam = cv2.VideoCapture(0)
        while True:
            ret_val, img = cam.read()
            if mirror:
                img = cv2.flip(img, 1)
            # img = cv2.resize(img, (960, 540))
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            faces = face_cascade.detectMultiScale(gray, 1.3, 5)
            for (x, y, w, h) in faces:
                cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)
                # roi_gray = gray[y:y+h, x:x+w]
                # roi_color = img[y:y + h, x:x + w]

            cv2.imshow('my webcam', img)
            if cv2.waitKey(1) == 27:
                break  # esc to quit
        cv2.destroyAllWindows()
