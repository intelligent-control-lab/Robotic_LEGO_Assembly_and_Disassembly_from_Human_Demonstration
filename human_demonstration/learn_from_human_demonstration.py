# This file constructs the robot executable task graph from the human demonstration video.
# Copyright (C) 2023

# Authors:
# Ruixuan Liu: ruixuanl@andrew.cmu.edu
# Changliu Liu : cliu6@andrew.cmu.edu

# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 3
# of the License, or (at your option) any later version.
 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

import cv2
import numpy as np
import os
import glob
import json

# Pre-learned (linear regression) pixel2grid mapping.
def calc_X_on_grid(pix):
    return round(pix * 0.0738 - 6.1233)

def calc_Y_on_grid(pix):
    return round(pix * (-0.0667) + 50.0323)

# Predefined brick type.
def get_brick_type(len_long, len_short):
    div = len_long / len_short
    if(round(div) == 3):
        return 3
    elif(round(div) == 8):
        return 4
    elif(round(div) == 4):
        return 8
    elif(round(div) == 2):
        return 9
    else:
        return 0
    

input_root = './human_demonstration/'
output_root = './config/assembly_tasks/'
# Download the video to the input_root folder.
# Video Link: https://drive.google.com/file/d/1B8meDY02TvG3Zd5gK1jp-HvyWr81xG2g/view?usp=share_link
vidcap = cv2.VideoCapture(input_root + 'RI.webm')
window_size = 15
img_diff_thres = 40
img_static_thres = 500

# Clear folders
if(not os.path.exists(input_root + 'img/')):
    os.mkdir(input_root + 'img/')
if(not os.path.exists(input_root + 'key_frames/')):
    os.mkdir(input_root + 'key_frames/')
if(not os.path.exists(output_root)):
    os.mkdir(output_root)

files = glob.glob(input_root + 'img/*')
for f in files:
    os.remove(f)
files = glob.glob(input_root + 'key_frames/*')
for f in files:
    os.remove(f)

# Get all frames
img_idx = 0
success, image = vidcap.read()
while(success):
    image = image[350:, 400:, :]
    cv2.imwrite(input_root + 'img/' + str(img_idx) + '.jpg', image)
    success, image = vidcap.read()
    img_idx += 1
num_frames = img_idx
print("Done extracting video! Got " + str(num_frames) + " frames!")

# Extract keyframes
key_frame_id = 0
is_keyframe = False
pre_is_keyframe = False

for i in range(window_size, num_frames):
    img_id1 = i
    img_id2 = i - window_size
    img1 = cv2.imread(input_root + 'img/' + str(img_id1) + '.jpg')
    img2 = cv2.imread(input_root + 'img/' + str(img_id2) + '.jpg')
    img_diff = cv2.cvtColor(cv2.subtract(img1, img2), cv2.COLOR_BGR2GRAY)
    ret, img_diff = cv2.threshold(img_diff, img_diff_thres, 255, cv2.THRESH_BINARY)

    diff_norm = np.linalg.norm(img_diff)
    if(diff_norm < img_static_thres):
        is_keyframe = True
    else:
        is_keyframe = False

    if(is_keyframe and not pre_is_keyframe):
        cv2.imwrite(input_root + 'key_frames/' + str(key_frame_id) + '.jpg', img1)
        key_frame_id += 1
    pre_is_keyframe = is_keyframe
print("Extracted " + str(key_frame_id) + " keyframes!")

# Construct task graph
num_keyframes = key_frame_id
empty_frame = cv2.imread(input_root + 'key_frames/0.jpg')
empty_frame = empty_frame[:, :empty_frame.shape[1] // 2, :]
task_graph = {}

for i in range(1, num_keyframes):
    img1 = cv2.imread(input_root + 'key_frames/' + str(i - 1) + '.jpg')
    img1 = img1[:, :img1.shape[1] // 2, :]
    img2 = cv2.imread(input_root + 'key_frames/' + str(i) + '.jpg')
    img2 = img2[:, :img2.shape[1] // 2, :]
    img_diff = cv2.subtract(img1, img2)
    ret, img_diff_thres = cv2.threshold(img_diff, 68, 255, cv2.THRESH_BINARY)
    img_diff_gray = cv2.cvtColor(img_diff_thres, cv2.COLOR_BGR2GRAY)
    ret, img_diff_gray = cv2.threshold(img_diff_gray, 20, 255, cv2.THRESH_BINARY)

    img = empty_frame
    min_c = 1000000
    max_c = 0
    max_r = 0
    min_r = 1000000
    for r in range(img.shape[0]):
        for c in range(img.shape[1]):
            if(img_diff_gray[r, c] > 0):
                img[r, c, :] = img2[r, c, :]
                if(r > max_r):
                    max_r = r
                if(r < min_r):
                    min_r = r
                if(c < min_c):
                    min_c = c
                if(c > max_c):
                    max_c = c

    len_y = abs(calc_Y_on_grid(max_r) - calc_Y_on_grid(min_r))
    len_x = abs(calc_X_on_grid(max_c) - calc_X_on_grid(min_c))
    if(len_y > len_x):
        orientation = 0
        len_x = abs(calc_X_on_grid((max_c-min_c) * 0.6 + min_c) - calc_X_on_grid(min_c))
        brick_type = get_brick_type(len_y, len_x)
    else:
        orientation = 1
        brick_type = get_brick_type(len_x, len_y)
    
    print("x: ", min_c, " ", calc_X_on_grid(min_c), " y: ", max_r, " ", calc_Y_on_grid(max_r), orientation, (len_y, len_x), brick_type)
    subtask = {"x":calc_X_on_grid(min_c), "y":calc_Y_on_grid(max_r), "z":1, "ori":orientation, "id":brick_type}
    task_graph[str(i)] = subtask
    cv2.imwrite(input_root + 'key_frames/' + str(i) + "_mask.jpg", img_diff_gray)

json_object = json.dumps(task_graph, indent=4)
with open(output_root + "RI.json", "w") as outfile:
    outfile.write(json_object)
