#!/usr/bin/python3

import cv2
import numpy as np
import os
import urllib.request


def manipulate():
    path = "/mnt/d/Verkehrsschilder/"
    folder = "pos11"

    print(os.path.join(path, folder))

    for (dir_path, dir_names, file_names) in os.walk(os.path.join(path, folder)):
        for file in file_names:
            print(os.path.join(dir_path, file))
            img = cv2.imread(os.path.join(dir_path, file), cv2.IMREAD_GRAYSCALE)
            dim = 50
            resized_image = cv2.cvtColor(cv2.resize(img, (dim, dim)), cv2.COLOR_GRAY2BGR)
            cv2.imwrite(os.path.join(path, folder, file), resized_image)


def download():
    neg_images_link = 'http://image-net.org/api/text/imagenet.synset.geturls?wnid=n02686568'
    neg_image_urls = urllib.request.urlopen(neg_images_link, timeout=5).read().decode()
    pic_num = 2626

    if not os.path.exists('neg'):
        os.makedirs('neg')

    for i in neg_image_urls.split('\n'):
        try:
            print(i)
            urllib.request.urlretrieve(i, "neg/" + str(pic_num) + ".jpg")
            img = cv2.imread("neg/" + str(pic_num) + ".jpg", cv2.IMREAD_GRAYSCALE)
            # should be larger than samples / pos pic (so we can place our image on it)
            resized_image = cv2.resize(img, (100, 100))
            cv2.imwrite("neg/" + str(pic_num) + ".jpg", resized_image)
            pic_num += 1

        except Exception as e:
            print(str(e))


def find_uglies():
    match = False
    for file_type in ['neg']:
        for img in os.listdir(file_type):
            for ugly in os.listdir('uglies'):
                try:
                    current_image_path = str(file_type)+'/'+str(img)
                    ugly = cv2.imread('uglies/'+str(ugly))
                    question = cv2.imread(current_image_path)
                    if ugly.shape == question.shape and not(np.bitwise_xor(ugly, question).any()):
                        print('That is one ugly pic! Deleting!')
                        print(current_image_path)
                        os.remove(current_image_path)
                except Exception as e:
                    print(str(e))


def create_pos_n_neg():
    for file_type in ['neg']:

        for img in os.listdir(file_type):

            if file_type == 'pos':
                line = file_type + '/' + img + ' 1 0 0 50 50\n'
                with open('info.dat', 'a') as f:
                    f.write(line)
            elif file_type == 'neg':
                line = file_type + '/' + img + '\n'
                with open('bg.txt', 'a') as f:
                    f.write(line)


# download()
# find_uglies()
create_pos_n_neg()