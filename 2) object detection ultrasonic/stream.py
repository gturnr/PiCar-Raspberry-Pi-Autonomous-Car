# Author - Guy Turner
# Date - 13/03/2018
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

kill_threads = []  # list used to store the ids of threads waiting to be terminated
distance_threshold = 30.0  # minimum distance range before the car stops
ultrasonic_distance = 0.0  # stores the current value of the ultrasonic sensor
speed = 1  # value specifying if the car is moving (1 = yes, 0 = no)


def thread_cameraserver():  # thread for receiving and processes the camera feed
    global speed
    server_socket = socket.socket()  # creates a web-socket server with the port 8000 for the camera feed
    server_socket.bind(('0.0.0.0', 8000))
    server_socket.listen(0)
    connection = server_socket.accept()[0].makefile('rb')  # accepts the first connection made to the web-socket
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
            img = cv2.flip(cv2.imdecode(file_bytes, cv2.IMREAD_COLOR), 1)
            # renders the current reading from the ultrasonic sensor onto the screen
            ultrasonic_text = str(round(ultrasonic_distance))
            cv2.putText(img, 'u-s dist: ' + ultrasonic_text, (0, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            if speed == 1:  # renders text onto the screen stating if the car is moving or stopped
                cv2.putText(img, 'moving', (0, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 1)
            else:
                cv2.putText(img, 'Stopped!', (0, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 1)

            cv2.imshow('image', img)  # outputs the processed image to a display window for the operator to view
            cv2.waitKey(1)
    finally:
        connection.close()  # close the connection to the Pi and terminate the web-socket
        server_socket.close()


def thread_ultrasonicserver():
    global speed, ultrasonic_distance
    s = socket.socket()
    s.bind(('0.0.0.0', 8001))
    s.listen(0)
    connection = s.accept()[0]
    # before the car is told to stop there must be 3 consecutive values from the US sensor under the treshhold
    validator = 0

    while True:
        if distance_feed.ident in kill_threads:  # statement to check if the main thread is exiting
            connection.close()
            sys.exit()
        try:
            message = str(connection.recv(1024), 'utf-8')  # receives the message from the Pi
            if message:  # checks if the message is not Null
                ultrasonic_distance = float(message)  # converts the message to a float
                if int(ultrasonic_distance) < distance_threshold:  # checks if the car is within the threshold distance
                    validator += 1  # valid frames is increased by 1
                else:
                    validator = 0  # valid frames is set to 0

                if validator >= 3:  # if there has been 3 or more valid frames in a row the car is told to stop
                    speed = 0
                else:  # else, the car is told to keep moving
                    speed = 1
        except:
            continue
    connection.close()


def thread_steering_output():
    global speed
    s = socket.socket()  # creates a web-socket server using port 8002 for the steering data
    s.bind(('0.0.0.0', 8002))
    s.listen(0)  # listens for a connection request from the Raspberry Pi
    connection = s.accept()[0]  # accepts the first attempted connection
    print('motor connected')

    while True:  # continuous loop until the thread is terminated
        if steering_output.ident in kill_threads:  # statement to check if the main thread is exiting
            connection.close()
            sys.exit()

        connection.send(str(speed).encode('utf-8'))  # sends the current speed value to the Pi
        time.sleep(0.1)  # limits the loop to only run ~10x a second

    connection.close()


if __name__ == '__main__':  # the script will only run if it is run directly
    camera_feed = threading.Thread(target=thread_cameraserver)  # defines the camera feed thread
    camera_feed.start()  # starts the camera feed thread
    distance_feed = threading.Thread(target=thread_ultrasonicserver)  # defines the ultrasonic data thread
    distance_feed.start()  # starts the ultrasonic data thread
    steering_output = threading.Thread(target=thread_steering_output)  # defines the steering output thread
    steering_output.start()  # starts the steering output thread

    try:
        while True:  # keeps the main thread running for a clean exit
            time.sleep(1)

    except(KeyboardInterrupt, SystemExit):  # Tells each thread to terminate at the beginning of the next loop
        kill_threads.append(camera_feed.ident)
        kill_threads.append(distance_feed.ident)
        kill_threads.append(steering_output.ident)
        print('threads safely terminated')
