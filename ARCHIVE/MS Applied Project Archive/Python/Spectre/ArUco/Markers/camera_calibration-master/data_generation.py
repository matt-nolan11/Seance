'''This script is for generating data
1. Provide desired path to store images.
2. Press 'c' to capture image and display it.
3. Press any button to continue.
4. Press 'q' to quit.
'''

import cv2

camera = cv2.VideoCapture(1)
ret, img = camera.read()


path = ("I:\\My Drive\\College Stuff\\S9 Fall 2022\\MAE 593\\Aruco Markers\\camera_calibration-master\\Calibration_Images\\c")
count = 0
while True:
    name = path + str(count)+".jpg"
    ret, img = camera.read()
    cv2.imshow("img", img)


    if cv2.waitKey(20) & 0xFF == ord('c'):
        cv2.imwrite(name, img)
        cv2.imshow("img", img)
        count += 1
        if cv2.waitKey(0) & 0xFF == ord('q'):

            break;
