"""
Authors:        Jan Füsting
Last edited:    10.09.2018
"""
import cv2
import os

from Misc import Misc


class Recognition:
    """
    This class handles the
    """
    def __init__(self):
        """
        Initialising
        """
        self.misc = Misc()

    def face_recognition(self):
        """
        Sample cascade to detect faces
        """
        cascade = cv2.CascadeClassifier(
            os.path.join(self.misc.project_root, 'dataset', 'haarcascade_frontalface_default.xml'))
        self.get_camera_image()

    def get_camera_image(self, mirror=True):
        """
        Shows live images with marked detections
        """
        self.misc.logger.debug("Show cam")

        class_01_cascade = cv2.CascadeClassifier(
            os.path.join(self.misc.project_root, 'dataset', '01_cascade.xml'))

        face_cascade = cv2.CascadeClassifier(
            os.path.join(self.misc.project_root, 'dataset', 'haarcascade_frontalface_default.xml'))

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
                self.write_text_on_image(img, "Faces", (x, y-5))

            # class_01 = class_01_cascade.detectMultiScale(gray, 1.3, 5)
            # for (x, y, w, h) in class_01:
            #     cv2.rectangle(img, (x, y), (x+w, y+h), (255, 255, 0), 2)
            #     self.write_text_on_image(img, "entry forbidden", (x, y-5))

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



