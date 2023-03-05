import paho.mqtt.client as mqtt
import time
import RPi.GPIO as gp  
from time import sleep  

o2 = 100

def on_connect(client,userdata,flags,rc):
    global flag_connected
    flag_connected = 1
    client_subscriptions(client)
    print('Connected to MQTT server')
    
def on_disconnect(client,userdata,rc):
    global flag_connected
    flag_connected = 0
    print('Disconnected from MQTT server')

def callback_esp32_sensor2(client,userdata,msg):
    global o2
    if msg.payload.decode('utf-8') == '-999':
        print('SpO2: ','Bad Read')
    
    else:
        print('SpO2: ',str(msg.payload.decode('utf-8')))
        o2 = int(msg.payload.decode('utf-8'))
    
def client_subscriptions(client):
    client.subscribe('esp32/#')

client = mqtt.Client("rpi_client1")
flag_connected = 0

client.on_connect = on_connect
client.on_disconnect = on_disconnect

client.message_callback_add('esp32/sensor2',callback_esp32_sensor2)

client.connect('localhost',1883)

client.loop_start()
client_subscriptions(client)
print('......client setup complete......')

#Initialize the fan motor
gp.setmode(gp.BOARD)  
gp.setup(12,gp.OUT)  
gp.setup(32,gp.OUT)  
pwm=gp.PWM(12,50)  
pwm1=gp.PWM(32,50)  
pwm.start(0)  
pwm1.start(0) 

while True:
    time.sleep(4)
    if o2 <= 75:
        print('Running fan at 100% duty cycle')
        pwm.ChangeDutyCycle(100) 
    elif o2 <= 85:
        pwm.ChangeDutyCycle(50) 
        print('Running fan at 50% duty cycle')
    else:
        pwm.ChangeDutyCycle(0) 

    if(flag_connected != 1):
        print('trying to connect MQTT server. .')



