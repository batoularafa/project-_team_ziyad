from vpython import *
import websocket
import cv2
import re
import requests
import numpy as np

location_tup = ()  # create an empty tuple to save the decoded qr code info
location_ls = []  # create an empty list to store each unique location tuple
location_done = []
img_name = 1  # give the captured image unique name

# creates an instance of the QRCodeDetector class
qcd = cv2.QRCodeDetector()

# Replace the below URL with your own. Make sure to add "/shot.jpg" at last.
url = "http://192.168.219.67:8080//shot.jpg"

ws = websocket.WebSocket()
IP = "192.168.219.218" #this IP is printed on the Serial port of ESP32.
ws.connect("ws://" + IP )

print("Connected to the server!!")

boundary = box(pos=vector(0, 0, 0), size=vector(400, 400, 0.1), color=color.white)
robot=box(pos=vector(-150,-150,0),axis=vector(1,0,0),size=vector(20,20,5),color=color.blue)
target=box(pos=vector(150,150,0),axis=vector(1,0,0),size=vector(10,10,5),color=color.red)
x_robot=-150
y_robot=-150
x_box=150
y_box=150
while True : 
    #read qrcode
    img_resp = requests.get(url, verify=False)
    img_arr = np.array(bytearray(img_resp.content), dtype=np.uint8)
    frame = cv2.imdecode(img_arr, -1)

    ret_qr, decoded_info, points, straight_qrcode = qcd.detectAndDecodeMulti(frame)

    # frame = imutils.resize(frame, width=1000, height=1800)
    frame = cv2.resize(frame, (1000, 700))
    cv2.imshow("stream", frame)

    if ret_qr:
        for s, p in zip(decoded_info, points):
            if s:
                # extract any number exists in the string
                # [-+]?: Matches an optional positive or negative sign(+ or -).
                # \d*: Matches all digits [0:9].
                # \.: Matches decimal point.
                # \d+: indicates the numbers may be of any length.
                location_tup = tuple(re.findall(r"[-+]?\d*\.\d+|\d+", s))
                # check if the location exists in the location list
                # if not then append
                if (location_tup not in location_ls) and (location_tup not in location_done):
                    location_ls.append(location_tup)
                    print(location_ls)
                    print("location tuple", location_tup)
                    print("x= ", location_tup[0])
                    print("location done", location_done)

            #     color = (0, 255, 0)  # if s is decoded then the square will be green
            # else:
            #     color = (0, 0, 255)  #else if s is not decoded right the square will be red
            # frame = cv2.polylines(frame, [p.astype(int)], True, color, 8)  #draw the square according to the corners of the qr code
    # cv2.imshow("qr", frame) # to display a window with the square

    # Press Esc key to exit
    if cv2.waitKey(1) == 27:
        cv2.destroyAllWindows()
        break
    msg = ws.recv()
    print(msg)
    crd_robot = msg.split(',')
    x_robot= (float(crd_robot[0])*100)-150
    y_robot= (float(crd_robot[1])*100)-150

    #recieve data from esp32 and print it
    if len(location_ls) >= 1:
        crd_box = location_ls[len(location_ls)-1]
        print(crd_box[0],crd_box[1])
        x_box = (float(crd_box[0])*100)-150
        y_box = (float(crd_box[1])*100)-150
        print(x_robot,y_robot)
        print(x_box,y_box)

        #move robot
    rate(80)
    robot.pos=vector(x_robot,y_robot,0)
    target.pos= vector(x_box,y_box,0)
        
ws.close()