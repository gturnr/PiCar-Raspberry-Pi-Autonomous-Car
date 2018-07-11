import pygame, socket, time, struct, io, cv2, sys
import threading
import numpy as np
# libraries used for this script are listed above.

pygame.init()  # initialises pygame, used for creating a user interface
pygame.font.init()

main_speed = 0
turning = 0

ultrasonic_reading = 0.0
current_frame = None
ip_address = '0.0.0.0'

# defines basic variables for use with Pygame such as fonts and colours
font = pygame.font.SysFont('Arial', 20)
boldfont = pygame.font.SysFont('Arial', 50)
speedfont = pygame.font.SysFont('Arial', 72)
directionfont = pygame.font.SysFont('Arial', 65)
white = (255, 255, 255)
black = (0, 0, 0)
red = (255, 0, 0)
green = (0, 255, 0)

# sets the screen dimensions
display_width = 860
display_height = 540

kill_threads = []
connected = []

# defines and imports the harr cascade file used for stop sign detection
stopsign_cascade = cv2.CascadeClassifier('assets/stopsign_classifier.xml')


def get_focal_length():  # function to calculate the focal length of the camera based on a presaved image
    img = cv2.imread('assets/images/stopsign.png')  # loads the test image
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  # converts the image to gray scale
    stopsigns = stopsign_cascade.detectMultiScale(gray, 1.3, 5)  # runs the haar cascade to detect the stop sign
    distance = 38.5  # distance sign is from the camera in the photo (cm)
    width = 3.0  # physical width of the stop sign (cm)
    for (x, y, w, h) in stopsigns:  # loops through the stop sign detections (in this photo only 1)
        focal = (w * distance) / width  # calculates the focal using the width in px of the stop sign in the photo
        return focal  # returns the generated focal length


focal_length = get_focal_length()  # calls the get_focal_length() function and assigns the value to a global variable


class Button:  # class used to create buttons in pygame
    def __init__(self, display, x, y, w, h, text, box_colour, text_colour, font_type):
        self.display = display
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        self.clickable = False
        self.box_colour = box_colour
        self.text_surf = font_type.render(text, True, text_colour)
        self.text_rect = self.text_surf.get_rect()
        self.text_rect.center = ((x + (w / 2)), (y + (h / 2)))
        self.time = time.time()

    def set_func(self, func):  # sets the function to call when the button is pressed
        self.clickable = True
        self.func = func

    def draw(self):  # function to render the button to the screen
        pygame.draw.rect(self.display, self.box_colour, [self.x, self.y, self.w, self.h])
        self.display.blit(self.text_surf, self.text_rect)

        if self.clickable:  # detects if the user has clicked the button
            mouse = pygame.mouse.get_pos()
            click = pygame.mouse.get_pressed()

            if self.x + self.w > mouse[0] > self.x and self.y + self.h > mouse[1] > self.y:
                if click[0] == 1:
                    if time.time() - self.time > 0.2:
                        self.time = time.time()
                        self.func()


class TextObject:  # class to create text objects in pygame
    def __init__(self, display, text, font_type, colour, x, y):
        title = font_type.render(text, True, colour)
        rect = title.get_rect()
        rect.left = x
        rect.top = y
        display.blit(title, rect)

### The following subroutines are called by buttons to alter the cars
#   speed and direction

def decrease_speed():
    global main_speed
    if main_speed > 0:
        main_speed -= 10


def increase_speed():
    global main_speed
    if main_speed < 100:
        main_speed += 10


def stop_car():
    global main_speed, turning
    main_speed = 0
    turning = 0


def turn_left():
    global turning
    if turning > -100:
        turning -= 10


def turn_right():
    global turning
    if turning < 100:
        turning += 10

###


def menu_interface():  # subroutine for pygame
    # images
    loading_template = pygame.image.load('assets/images/loading_screen.jpg')

    # define pygame window characteristics
    display_obj = pygame.display.set_mode((display_width, display_height))
    pygame.display.set_caption('Pi Bot')
    clock = pygame.time.Clock()

    # defines control buttons
    left_button = Button(display_obj, 652, 195, 50, 50, '<', (255, 238, 0), black, directionfont)
    left_button.set_func(turn_left)
    right_button = Button(display_obj, 805, 195, 50, 50, '>', (0, 148, 255), black, directionfont)
    right_button.set_func(turn_right)
    increase_button = Button(display_obj, 805, 85, 50, 50, '+', green, black, directionfont)
    increase_button.set_func(increase_speed)
    decrease_button = Button(display_obj, 652, 85, 50, 50, '-', red, black, directionfont)
    decrease_button.set_func(decrease_speed)
    stop_button = Button(display_obj, 652, 265, 205, 50, 'STOP', red, black, boldfont)
    stop_button.set_func(stop_car)

    running = True

    # loop that runs until the user tries to exit the program
    while running:
        # handles exit
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        display_obj.fill(white)  # fills the background with white

        # draws a placeholder black square for the camera feed
        pygame.draw.rect(display_obj, black, [5, 55, 640, 480])

        # display camera feed
        try:
            # gets the most recent camera frame and renders it to the screen
            feed = pygame.surfarray.make_surface(current_frame)
            feed = pygame.transform.rotate(feed, -90)
            display_obj.blit(pygame.transform.flip(feed, True, False), (5, 55))
        except:
            # if the camera is not connected a placeholder image is used
            display_obj.blit(loading_template, (5, 55))

        # defines the speed and direction buttons
        speed_box = Button(display_obj, 702, 85, 105, 50, str(main_speed), (188, 188, 188), black, boldfont)
        direction_box = Button(display_obj, 702, 195, 105, 50, str(turning), (188, 188, 188), black, boldfont)

        # renders all of the buttons to the screen
        left_button.draw()
        right_button.draw()
        direction_box.draw()
        stop_button.draw()
        increase_button.draw()
        decrease_button.draw()
        speed_box.draw()

        # renders all of the text objects to the screen
        title = boldfont.render("PiCar", True, black)
        display_obj.blit(title, (380, 5))
        TextObject(display_obj, "Speed", font, black, 650, 55)
        TextObject(display_obj, "Direction", font, black, 650, 165)
        TextObject(display_obj, "Ultrasonic Distance", font, black, 650, 335)
        TextObject(display_obj, "Pi IP Address", font, black, 650, 410)
        TextObject(display_obj, ip_address, font, black, 650, 435)
        TextObject(display_obj, str(round(ultrasonic_reading, 1)), font, black, 650, 360)

        # checks if the car is connected by websocket
        if len(connected) == 3:
            TextObject(display_obj, "Connected", font, green, 650, 485)
        else:
            TextObject(display_obj, "Disconnected", font, red, 650, 485)

        pygame.display.update()  # updates the current pygame frame
        clock.tick(60)  # sets the output target fps to 60

    pygame.quit()  # after the loop the program terminates
    sys.exit()


def thread_cameraserver():  # thread for receiving and processes the camera feed
    global current_frame, connected
    server_socket = socket.socket()  # creates a web-socket server with the port 8000 for the camera feed
    server_socket.bind(('0.0.0.0', 8000))
    server_socket.listen(0)
    connection = server_socket.accept()[0].makefile('rb')  # accepts the first connection made to the web-socket
    connected.append(True)
    try:
        while True:
            if camera_feed.ident in kill_threads:  # thread termination handling
                connection.close()
                server_socket.close()
                sys.exit()
            image_len = struct.unpack('<L', connection.read(struct.calcsize('<L')))[0]
            if not image_len:  # if there is no data terminate the loop
                break
            # the following lines of code interpret and convert the image
            # from the camera feed
            image_stream = io.BytesIO()
            image_stream.write(connection.read(image_len))
            image_stream.seek(0)
            file_bytes = np.asarray(bytearray(image_stream.read()), dtype=np.uint8)
            img = cv2.flip(cv2.imdecode(file_bytes, cv2.IMREAD_COLOR), 1)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  # converts the image to gray scale
            # using the cascade file the program analyses the image to try and identify any stop signs
            stopsigns = stopsign_cascade.detectMultiScale(gray, 1.3, 5)
            for (x, y, w, h) in stopsigns:  # draws a rectangle around any identified stop signs
                distance_to_sign = (3.0 * focal_length) / w  # calculates the approximate distance from car
                cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)  # draws a box around the detected stop sign
                cv2.putText(img, str(round(distance_to_sign, 1)), (x, y + h), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (0, 0, 255), 1)
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            current_frame = img  # sets the current frame for pygame to the newly analysed frame
    except Exception:  # if the camera is not connected clear the current frame
        current_frame = None

    finally:
        connection.close()  # close the connection to the Pi and terminate the web-socket
        server_socket.close()


def thread_ultrasonicserver():  # thread used to receive us data from the car
    global ultrasonic_reading, ip_address, connected
    s = socket.socket()
    s.bind(('0.0.0.0', 8001))
    s.listen(0)
    connection = s.accept()[0]
    ip_address = str(connection.getpeername()[0])  # gets the ip address of the raspberry pi from the connection
    connected.append(True)

    while True:  # loop whilst connected to the robot
        if distance_feed.ident in kill_threads:  # thread termination handling
            connected.pop()
            connection.close()
            sys.exit()
        try:
            message = str(connection.recv(1024), 'utf-8')  # receive the distance reading from the pi
            if message:
                ultrasonic_reading = float(message)  # convert the message to a float value
        except Exception:
            continue  # if no data received, try again


def thread_steering_output():  # thread used to sent speed and steering data to the car
    global connected
    s = socket.socket()
    s.bind(('0.0.0.0', 8002))
    s.listen(0)
    connection = s.accept()[0]
    connected.append(True)
    try:
        while True:
            if steering_output.ident in kill_threads:  # thread termination handling
                connection.close()
                sys.exit()
            # converts the speed and steering data into a list for the car
            if turning == 0:
                speed = [main_speed, main_speed]
            elif turning > 0:
                speed = [main_speed, main_speed - (main_speed*(turning/100))]
            elif turning < 0:
                speed = [main_speed - (main_speed*abs(turning/100)), main_speed]

            connection.send(str(speed).encode('utf-8'))  # sends the data to the car
            time.sleep(0.1)  # creates a break delay, data sent 10 times a second
    except Exception:
        connected.pop()
    connection.close()


# launches the program when run directly
if __name__ == '__main__':
    try:  # creates each thread for the camera, us and steering
        camera_feed = threading.Thread(target=thread_cameraserver)
        camera_feed.start()
        distance_feed = threading.Thread(target=thread_ultrasonicserver)
        distance_feed.start()
        steering_output = threading.Thread(target=thread_steering_output)
        steering_output.start()
        menu_interface()  # calls the pygame subroutines

    except(KeyboardInterrupt, SystemExit):  # Tells each thread to terminate at the beginning of the next loop
        kill_threads.append(camera_feed.ident)
        kill_threads.append(distance_feed.ident)
        kill_threads.append(steering_output.ident)
        print('threads safely terminated')
