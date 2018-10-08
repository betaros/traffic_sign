"""
Authors:        Jan Füsting
Last edited:    10.09.2018
"""
import cv2
import os

from src.Misc import Misc


class Recognition:
    """
    This class handles the
    """
    def __init__(self):
        """
        Initialising
        """
        self.misc = Misc()

    def get_camera_image(self, mirror=True):
        """
        Shows live images with marked detections
        """
        self.misc.logger.debug("Show cam")

        show_face = False
        show_class1 = False
        show_class2 = False
        show_class3 = True
        show_class4 = False
        show_class5 = True
        show_class6 = False
        show_class7 = False
        show_class8 = False
        show_class9 = False
        show_class10 = False
        show_class11 = False

        if show_class1:
            class_01_cascade = cv2.CascadeClassifier(
                os.path.join(self.misc.project_root, 'dataset', 'cascade_entry_forbidden.xml'))

        if show_class2:
            class_02_cascade = cv2.CascadeClassifier(
                os.path.join(self.misc.project_root, 'dataset', 'cascade_pedestrians.xml'))

        if show_class3:
            class_03_cascade = cv2.CascadeClassifier(
                os.path.join(self.misc.project_root, 'dataset', 'cascade_turn_right.xml'))

        if show_class4:
            class_04_cascade = cv2.CascadeClassifier(
                os.path.join(self.misc.project_root, 'dataset', 'cascade_main_road.xml'))

        if show_class5:
            class_05_cascade = cv2.CascadeClassifier(
                os.path.join(self.misc.project_root, 'dataset', 'cascade_turn_left.xml'))

        if show_class6:
            class_06_cascade = cv2.CascadeClassifier(
                os.path.join(self.misc.project_root, 'dataset', 'cascade_warning.xml'))

        if show_class7:
            class_07_cascade = cv2.CascadeClassifier(
                os.path.join(self.misc.project_root, 'dataset', 'cascade_no_parking.xml'))

        if show_class8:
            class_08_cascade = cv2.CascadeClassifier(
                os.path.join(self.misc.project_root, 'dataset', 'cascade_bus_stop.xml'))

        if show_class9:
            class_09_cascade = cv2.CascadeClassifier(
                os.path.join(self.misc.project_root, 'dataset', 'cascade_crossing.xml'))

        if show_class10:
            class_10_cascade = cv2.CascadeClassifier(
                os.path.join(self.misc.project_root, 'dataset', 'cascade_slippery.xml'))

        if show_class11:
            class_11_cascade = cv2.CascadeClassifier(
                os.path.join(self.misc.project_root, 'dataset', 'cascade_road_closed.xml'))

        if show_face:
            face_cascade = cv2.CascadeClassifier(
                os.path.join(self.misc.project_root, 'dataset', 'haarcascade_frontalface_default.xml'))

        cam = cv2.VideoCapture(0)
        while True:
            ret_val, img = cam.read()
            if mirror:
                img = cv2.flip(img, 1)
            # img = cv2.resize(img, (960, 540))
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            if show_face:
                faces = face_cascade.detectMultiScale(gray, 1.3, 5)
                for (x, y, w, h) in faces:
                    cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)
                    self.write_text_on_image(img, "Faces", (x, y-5))

            if show_class1:
                class_01 = class_01_cascade.detectMultiScale(gray, 1.3, 5)
                for (x, y, w, h) in class_01:
                    cv2.rectangle(img, (x, y), (x+w, y+h), (255, 0, 0), 2)
                    self.write_text_on_image(img, "entry forbidden", (x, y-5))

            if show_class2:
                class_02 = class_02_cascade.detectMultiScale(gray, 1.3, 5)
                for (x, y, w, h) in class_02:
                    cv2.rectangle(img, (x, y), (x + w, y + h), (255, 128, 0), 2)
                    self.write_text_on_image(img, "pedestrians", (x, y - 5))

            if show_class3:
                class_03 = class_03_cascade.detectMultiScale(gray, 1.3, 5)
                for (x, y, w, h) in class_03:
                    cv2.rectangle(img, (x, y), (x+w, y+h), (128, 255, 0), 2)
                    self.write_text_on_image(img, "turn right", (x, y-5))

            if show_class4:
                class_04 = class_04_cascade.detectMultiScale(gray, 1.3, 5)
                for (x, y, w, h) in class_04:
                    cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 255), 2)
                    self.write_text_on_image(img, "main road", (x, y - 5))

            if show_class5:
                class_05 = class_05_cascade.detectMultiScale(gray, 1.3, 5)
                for (x, y, w, h) in class_05:
                    cv2.rectangle(img, (x, y), (x+w, y+h), (0, 128, 0), 2)
                    self.write_text_on_image(img, "turn left", (x, y-5))

            if show_class6:
                class_06 = class_06_cascade.detectMultiScale(gray, 1.3, 5)
                for (x, y, w, h) in class_06:
                    cv2.rectangle(img, (x, y), (x + w, y + h), (0, 128, 128), 2)
                    self.write_text_on_image(img, "warning", (x, y - 5))

            if show_class7:
                class_07 = class_07_cascade.detectMultiScale(gray, 1.3, 5)
                for (x, y, w, h) in class_07:
                    cv2.rectangle(img, (x, y), (x+w, y+h), (255, 255, 128), 2)
                    self.write_text_on_image(img, "no parking", (x, y-5))

            if show_class8:
                class_08 = class_08_cascade.detectMultiScale(gray, 1.3, 5)
                for (x, y, w, h) in class_08:
                    cv2.rectangle(img, (x, y), (x + w, y + h), (255, 255, 0), 2)
                    self.write_text_on_image(img, "bus stop", (x, y - 5))

            if show_class9:
                class_09 = class_09_cascade.detectMultiScale(gray, 1.3, 5)
                for (x, y, w, h) in class_09:
                    cv2.rectangle(img, (x, y), (x+w, y+h), (255, 255, 0), 2)
                    self.write_text_on_image(img, "crossing", (x, y-5))

            if show_class10:
                class_10 = class_10_cascade.detectMultiScale(gray, 1.3, 5)
                for (x, y, w, h) in class_10:
                    cv2.rectangle(img, (x, y), (x + w, y + h), (255, 255, 0), 2)
                    self.write_text_on_image(img, "slippery", (x, y - 5))

            if show_class11:
                class_11 = class_11_cascade.detectMultiScale(gray, 1.3, 5)
                for (x, y, w, h) in class_11:
                    cv2.rectangle(img, (x, y), (x + w, y + h), (255, 255, 0), 2)
                    self.write_text_on_image(img, "road closed", (x, y - 5))

            cv2.imshow('Webcam', img)
            if cv2.waitKey(1) == 27:
                break  # esc to quit
        cv2.destroyAllWindows()

        self.misc.logger.debug("Exit cam")

    @staticmethod
    def write_text_on_image(img, message, bottom_left_corner_of_text=(10, 500)):
        """
        Writes text above the recognized field

        :param img:
        :param message:
        :param bottom_left_corner_of_text:
        :return:
        """
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 1
        font_color = (0, 255, 0)
        line_type = 2

        cv2.putText(img, message,
                    bottom_left_corner_of_text,
                    font,
                    font_scale,
                    font_color,
                    line_type)



