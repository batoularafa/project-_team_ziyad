import websocket
import pygame
 

#connect to the websocket server
ws = websocket.WebSocket()
IP = "192.168.219.218" #this IP is printed on the Serial port of ESP32.
ws.connect("ws://" + IP )

print("Connected to the server!!")

x = 0
y = 0
pygame.joystick.init() #detect controller
joystick_count = pygame.joystick.get_count()

if joystick_count == 0:
    print("No joysticks found.")
else:
     joysticks = [pygame.joystick.Joystick(x) for x in range(joystick_count)]
     print(joysticks)
     pygame.init()
     while True:
        
       # try:
        #    text = ws.recv()
         #   print(text)
        #except:
         #   print("error")

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                break
            if event.type == pygame.JOYBUTTONDOWN:
                if event.button == 2:
                    message = "G"
                    ws.send(message)
                  #  response = ws.recv()
                   # print("Received:", response)
                elif event.button == 3:
                    message = "R"
                    ws.send(message)
                #    response = ws.recv()
                 #   print("Received:", response)
                
            if event.type == pygame.JOYAXISMOTION:
                if event.axis == 0:
                    x = event.value
                    if event.value > 0.5:
                        #print("right")
                        message = "D"
                        ws.send(message)
                   #     response = ws.recv()
                    #    print("Received:", response)
                    elif event.value < -0.5:
                        #print("left")
                        message = "A"
                        ws.send(message)
                    #    response = ws.recv()
                     #   print("Received:", response)
                    elif abs(x)<0.1 and abs(y)<0.1:
                        message = "F"
                        ws.send(message)
                     #   response = ws.recv()
                    #    print("Received:", response)

            
                elif event.axis == 1:
                    y = event.value
                    if event.value > 0.5:
                        #print("back")
                        message = "S"
                        ws.send(message)
                    #    response = ws.recv()
                     #   print("Received:", response)
                    elif event.value < -0.5:
                        #print("forward")
                        message = "W"
                        ws.send(message)
                   #     response = ws.recv()
                    #    print("Received:", response)
                    elif abs(x)<0.1 and abs(y)<0.1:
                        message = "F"
                        ws.send(message)
                     #   response = ws.recv()
                      #  print("Received:", response)

                

pygame.quit()
ws.close()

'''while True:
    #connect to the websocket server
    ws = websocket.WebSocket()
    IP = "192.168.1.4" #this IP is printed on the Serial port of ESP32.
    ws.connect("ws://" + IP )

    print("Connected to the server!!")

    mssg = input("Enter something: ")
    ws.send(mssg)
    # wait for the server to respond and print it.
    result = ws.recv()
    print("Received: ", result)

#close the connection
ws.close()'''