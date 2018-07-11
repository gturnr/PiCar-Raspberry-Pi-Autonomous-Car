# Author - Guy Turner
# Date - 24/04/2018
# Project - EPQ Autonomous Vehicles Project 2018

import sys, time, threading, io, socket, struct, cv2
import numpy as np

# the above libraries are all used throughout the script
# sys - used to terminate the program upon completion
# time - used to create delays
# threading - used to create multiple threads for the program to 'multitask'
# io - used to handle the bitstream from the camera feed
# socket/struct - used to create and manage web-sockets between the Pi and Computer
# CV2 - used for image manipulation and processing
# numpy - used to convert the camera stream into an array

stop_sign_threshold = 40.0  # distance in cm from stop sign for car to stop
kill_threads = []  # list used to store the ids of threads waiting to be terminated
speed = 1  # value specifying if the car is moving (1 = yes, 0 = no)
stopsign_cascade = cv2.CascadeClassifier('stopsign_classifier.xml')  # location of the file used for classification


def get_focal_length():  # function to calculate the focal length of the camera based on a presaved image
    img = cv2.imread('stopsign.png')  # loads the test image
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  # converts the image to gray scale
    stopsigns = stopsign_cascade.detectMultiScale(gray, 1.3, 5)  # runs the haar cascade to detect the stop sign
    distance = 38.5  # distance sign is from the camera in the photo (cm)
    width = 3.0  # physical width of the stop sign (cm)
    for (x, y, w, h) in stopsigns:  # loops through the stop sign detections (in this photo only 1)
        focal = (w * distance) / width  # calculates the focal using the width in px of the stop sign in the photo
        return focal  # returns the generated focal length


focal_length = get_focal_length()  # calls the get_focal_length() function and assigns the value to a global variable


def thread_cameraserver():  # thread for receiving and processes the camera feed
    global speed
    server_socket = socket.socket()  # creates a web-socket server using port 8000 for the camera feed
    server_socket.bind(('0.0.0.0', 8000))
    server_socket.listen(0)
    connection = server_socket.accept()[0].makefile('rb')  # accepts the first connection made to the web-socket
    hold_frames = 0  # number of frames it has been detected it is clear to go after stopping
    try:
        while True:
            # statement to check if the main thread has requested the thread to terminate
            if camera_feed.ident in kill_threads:
                    connection.close()
                    server_socket.close()
                    sys.exit()
            
            image_len = struct.unpack('<L', connection.read(struct.calcsize('<L')))[0]
            if not image_len:  # if there is no data terminate the loop
                break
            
            image_stream = io.BytesIO()  # this block processes the byte stream from the Pi camera into a CV2 image
            image_stream.write(connection.read(image_len))
            image_stream.seek(0)
            # uses np to create an array containing the binary representation of the image
            file_bytes = np.asarray(bytearray(image_stream.read()), dtype=np.uint8)
            img = cv2.flip(cv2.imdecode(file_bytes, cv2.IMREAD_COLOR),1)

            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  # converts the image to gray scale
            # using the cascade file the program analyses the image to try and identify any stop signs
            stopsigns = stopsign_cascade.detectMultiScale(gray, 1.3, 5)

            distance_to_sign = 50  # sets a default value before detection to prevent any initial errors
            
            for (x, y, w, h) in stopsigns:  # draws a rectangle around any identified stop signs
                distance_to_sign = (3.0 * focal_length) / w  # calculates the approximate distance from car
                cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)  # draws a box around the detected stop sign
                cv2.putText(img, str(round(distance_to_sign, 1)), (x,y+h), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1)

            # checks that the car is less than 35cm away from the sign
            if len(stopsigns) != 0 and distance_to_sign < stop_sign_threshold:
                speed = 0  # sets the Pi car's speed to 0 (stopped)
                hold_frames = 0  # clears the number of frames detected it is clear for
            else:
                hold_frames += 1  # increases the number of clear frames by 1
                if hold_frames > 5:  # if the number of clear frames is more than 5
                    speed = 1  # sets the Pi car's speed to 1 (moving)

            if speed == 1:  # adds a text box to the image feed stating if the car is currently moving or stopped
                cv2.putText(img, 'moving',(0, 50),cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,0),1)
            else:
                cv2.putText(img, 'Stopped!', (0, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 1)

            cv2.imshow('image', img)  # outputs the processed image to a display window for the operator to view
            
            cv2.waitKey(1)
    finally:
        connection.close()  # close the connection to the Pi and terminate the web-socket
        server_socket.close()


def thread_steering_output():
    global speed
    s = socket.socket()  # creates a web-socket server using port 8002 for the steering data
    s.bind(('0.0.0.0', 8002))
    s.listen(0)  # listens for a connection request from the Raspberry Pi
    connection = s.accept()[0]  # accepts the first attempted connection
    print('motor connected')

    while True: # continuous loop until the thread is terminated
        # statement to check if the main thread has requested the thread to terminate
        if steering_output.ident in kill_threads:
            connection.close()
            sys.exit()

        connection.send(str(speed).encode('utf-8'))  # sends the current speed value to the Pi
        time.sleep(0.1)  # limits the loop to only run ~10x a second

    connection.close()
        

if __name__ == '__main__':  # the script will only run if it is run directly
    camera_feed = threading.Thread(target=thread_cameraserver)  # defines the camera feed thread
    camera_feed.start()  # starts the camera feed thread
    steering_output = threading.Thread(target=thread_steering_output)  # defines the steering output thread
    steering_output.start()  # starts the steering output thread

    try:
        while True:  # keeps the main thread running for a clean exit
            time.sleep(1)

    except(KeyboardInterrupt, SystemExit):  # Tells each thread to terminate at the beginning of the next loop
        kill_threads.append(camera_feed.ident)
        kill_threads.append(steering_output.ident)
        print('threads safely terminated')

