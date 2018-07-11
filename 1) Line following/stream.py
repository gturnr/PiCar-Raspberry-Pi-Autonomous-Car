# Author - Guy Turner
# Date - 12/04/2018
# Project - EPQ Autonomous Vehicles Project 2018

import sys, time, threading, io, socket, struct, cv2
import numpy as np

# the above libraries are all used throughout the script
# sys - used to terminate the program upon completion
# time - used to create delays
# threading - used to create multiple threads for the program to 'multitask'
# io - used to handle the bitstream from the camera feed
# socket/struct - to create and manage web-sockets between the Pi and Computer
# CV2 - for image manipulation and processing
# numpy - to convert the camera stream into an array
# math - for trig functions and calculations made for the line detection

kill_threads = []  # list to store the ids of threads waiting to be terminated

ultrasonic_distance = 0.0  # stores the current value of the ultrasonic sensor
distance_threashold = 30  # minimum distance range before the car stops
last_direction = 'left'  # sets which direction the car should start moving
default_power = 20  # default motor speed (out of 100)
lr_power = [default_power, default_power]  # sets the initial speed

def thread_cameraserver():
    global last_direction, lr_power
    # Start a socket listening for connections on 0.0.0.0:8000 (0.0.0.0 means
    # all interfaces)
    server_socket = socket.socket()
    server_socket.bind(('0.0.0.0', 8000))
    server_socket.listen(0)

    fps = 0
    totalframes = 0
    firstframe = True

    # Accept a single connection and make a file-like object out of it
    connection = server_socket.accept()[0].makefile('rb')
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
            file_bytes = np.asarray(bytearray(image_stream.read()), dtype=np.uint8)
            img = cv2.flip(cv2.imdecode(file_bytes, cv2.IMREAD_COLOR),1)

            cropped_img = img[360:480, 0:640] # crops the image from the bottom up to 240px, creating a 240x640 image

            gray = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2GRAY)  # converts the RGB image to gray-scale
            blurred = cv2.GaussianBlur(gray, (9, 9), 0)  # blurs the image to reduce detection of unwanted edges
            edged = cv2.Canny(blurred, 20, 50)  # performs canny edge detection on the image to identify distinct edges

            cv2.imshow('edged', edged)  # displays the processed image

            # variables for Hough Line Transform
            THRESHOLD = 9
            MAXLINEGAP = 10
            MINLENGTH = 40
            # creates a list of all detected lines using the Hough Line Transform function
            lines = cv2.HoughLinesP(
                edged,1,np.pi/180,THRESHOLD, np.array([]), MINLENGTH,MAXLINEGAP)
            midx = 320  # half the width of the feed

            # draws midline to show desired path
            cv2.line(cropped_img,(320,0),(320,240),(255,0,0),2)

            # Draw lines on input image
            if lines is not None:
                # gets the x and y co-ordinates for the first line detected
                for x1,y1,x2,y2 in lines[0]:
                    # finds the furthest x,y co-ord to combat issues with
                    # x1,x2 being upside down
                    if y1 < y2:
                        x, y = x1, y1
                    else:
                        x, y = x2, y2

                    # checks which side the x co-ord is on and determines which direction the car needs to move in
                    if midx < x:
                        last_direction = 'left'
                        force = x - midx

                    elif midx > x:
                        last_direction = 'right'
                        force = midx - x

                    # determines the turning force of the car dependant on the distance x is from the desired point
                    turning_variance=default_power-1.5*(default_power/midx)*force
                    if turning_variance < 0:
                        turning_variance = 0

                    # sets the values of the steering list to the values calculated above
                    if last_direction == 'right':
                        lr_power[0] = turning_variance
                        lr_power[1] = default_power
                    elif last_direction == 'left':
                        lr_power[0] = default_power
                        lr_power[1] = turning_variance

                    # adds text to the display to inform the user which way the car is turning
                    cv2.putText(img, 'turning ' + last_direction,(500,20),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),1)
                    cv2.putText(img, str(round(lr_power[0], 1))+' | ' + str(round(lr_power[1], 1)), (500,40), cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),1)

                    # renders the desired and current path of the car on the display
                    cv2.line(cropped_img,(x1,y1),(x2,y2),(0,255,0),2)
                    cv2.line(cropped_img,(midx,240),(x, y),(255,0,255),2)
            # if no line is detected:
            else:
                # adjust the direction to counter the loss of the line and attempt to refind the line
                if last_direction == 'right':
                    lr_power[0] = 0
                    lr_power[1] = default_power
                elif last_direction == 'left':
                    lr_power[0] = default_power
                    lr_power[1] = 0

            #fps calculator
            totalframes +=1
            if firstframe:
                starttime = time.time()
                firstframe = False
            else:
                # render the frames epr second count to the display (for connection and performance validation)
                fps = totalframes/(time.time()-starttime)
                cv2.putText(img, str(round(fps, 1)), (0, 50), cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,255),2)

            # output the displays to the user
            cv2.imshow('image', img)
            cv2.imshow('cropped', cropped_img)
            cv2.waitKey(1)
    finally:
        # on exit, closes the connection and the websocket
        connection.close()
        server_socket.close()


def thread_steering_output():
    global lr_power
    s = socket.socket()  # creates a web-socket server using port 8002 for the steering data
    s.bind(('0.0.0.0', 8002))
    s.listen(0)  # listens for a connection request from the Raspberry Pi
    connection = s.accept()[0]  # accepts the first attempted connection
    connection.send(str([0, 0]).encode('utf-8'))  # wait and tell the car to start stationery

    time.sleep(2)
    while True:  # continuous loop until the thread is terminated
        if steering_output.ident in kill_threads:  # statement to check if the main thread is exiting
                    connection.close()
                    sys.exit()

        connection.send(str(lr_power).encode('utf-8'))  # sends the current speed value to the Pi
        time.sleep(0.2)

    connection.close()
    # every frame update send the new angles


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
