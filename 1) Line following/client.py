# Author - Guy Turner
# Date - 12/04/2018
# Project - EPQ Autonomous Vehicles Project 2018

import io, socket, struct, time, sys, picamera, threading, os
import RPi.GPIO as gpio

# the above libraries are all used throughout the script
# io - used to handle the bitstream from the camera feed
# socket/struct - used to create web-sockets between the Pi and Computer
# time - used to create delays
# sys - used to terminate the program upon completion
# picamera - used to connect to the Raspberry Pi camera module
# threading - used to create multiple threads for the program to 'multitask'
# os - used to get environment variables
# RPi.GPIO - used to control the motors

server_ip = os.getenv('AUTOSERVERIP', '192.168.10.96')
cam_port = 8000
motor_port = 8002

# gpio pins for motors
forward_right_pin = 23
forward_left_pin = 22

# used for termination of threads
kill_threads = []

# GPIO configuration for us sensor and motors
gpio.setmode(gpio.BCM)
gpio.setup(forward_left_pin, gpio.OUT)
gpio.setup(forward_right_pin, gpio.OUT)


# thread streaming camerafeed over web-socket
def thread_camerafeed():
    print('camerafeed initialised...')
    camera_socket = socket.socket()  # creates a websocket connection
    camera_socket.connect((server_ip, cam_port))  # connects to the server
    print('camerafeed connected to server')
    # creates a stream to the socket as a binary 'file'
    connection = camera_socket.makefile('wb')
    try:
        # assigns the pi camera module to the variable camera
        with picamera.PiCamera() as camera:
            # sets the camera resolution to 640x480
            camera.resolution = (640, 480)
            camera.framerate = 10  # sets the frame rate to 10
            # inverts the vertical pixels from the camera
            camera.vflip = True
            time.sleep(2)  # waits 2 seconds to allow the camera to settle
            start = time.time()  # collects the starting time of the stream
            print('stream commenced at ' + str(time.ctime(int(start))))
            stream = io.BytesIO()
            # Use the video-port for captures...
            for foo in camera.capture_continuous(stream,'jpeg',use_video_port=True):
                # checks if the thread needs to terminate
                if camera_feed.ident in kill_threads:
                    sys.exit()

                # the following block reads the stream data from the camera
                # and then sends it to the server
                connection.write(struct.pack('<L', stream.tell()))
                connection.flush()  # clears the websocket ready for next frame
                stream.seek(0)
                connection.write(stream.read())  # sends the image data
                stream.seek(0)
                stream.truncate()

        # finally sends a null message to the server indicating to end the
        # connection
        connection.write(struct.pack('<L', 0))
    except Exception as e:  # catches any errors in the loop
        print('Camerafeed terminated - ' + str(e))

    finally:
        try:
            connection.close()  # closes the connection to the server
            camera_socket.close()
        except Exception as e:
            # catches any error incase the connection was prematurely terminated
            print('camerafeed already closed' + str(e))


def thread_motorcontrol():  # thread to control the motor speed
    print('motor control initialised...')
    left = gpio.PWM(forward_left_pin, 50)  # defines left motor PWM control
    right = gpio.PWM(forward_right_pin, 50)  # defines right motor PWM control
    motor_socket = socket.socket()  # creates a websocket connection
    motor_socket.connect((server_ip, motor_port))  # connects to the server
    print("connected to motor control server")
    left.start(0)  # sets the duty cycle of the left wheels to 0
    right.start(0)  # sets the duty cycle of the right wheels to 0

    while True:  # loop until the tjread terminates
        if motor_feed.ident in kill_threads:  # checks if thread needs to end
            connection.close()
            sys.exit()

        try:  # tries to receive a message if there is one
            # reads the message contents
            message = motor_socket.recv(1024).decode('utf-8')
            move = eval(message)  # converts the message to an integer
            left.ChangeDutyCycle(int(move[0]))
            right.ChangeDutyCycle(int(move[1]))

        except:  # if there is no message pass and try again
            continue

        time.sleep(0.1)  # allows the loop to run at ~10x a second


if __name__ == '__main__':  # runs when program is launched
    # defines and starts the camera streaming thread
    camera_feed = threading.Thread(target=thread_camerafeed)
    camera_feed.start()
    # defines and starts the motor control thread
    motor_feed = threading.Thread(target=thread_motorcontrol)
    motor_feed.start()

    try:
        while True:  # The following block of code watches if each thread has
                     # died (lost connection to the server) and
                     # attempts to restart it
            if not camera_feed.isAlive():
                print('restarting camera_feed')
                camera_feed = threading.Thread(target=thread_camerafeed)
                camera_feed.start()

            if not motor_feed.isAlive():
                print('restarting distance_feed')
                motor_feed = threading.Thread(target=thread_motorcontrol)
                motor_feed.start()

            time.sleep(1)
    # exception ran when the program is exited by the user
    except(KeyboardInterrupt, SystemExit):
        kill_threads.append(camera_feed.ident)  # tells each thread to terminate
        kill_threads.append(motor_feed.ident)
        print('threads safely terminated')
        gpio.cleanup()  # finally cleans the gpio ports and exits
