"""
Group 31 - The Cloud

SafeGuard Source Code

"""

import machine
import ads1x15
import network
import time
import math
import ujson
from umqtt.simple import MQTTClient
from micropython import const

"""
Define constants (e.g. addresses)
"""

accelx_threshold = 1.3;
accelz_threshold = 15.0;

wifi_ssid = "EEERover";
wifi_passw = "exhibition";

mqtt_server = "192.168.0.10";
mqtt_topic = "esys/thecloud";

LIS3DH_ADDR = const(24);
LIS3DH_REG_WHOAMI = const(0x0F);
LIS3DH_REG_CTRL1 = const(0x20);
LIS3DH_REG_CTRL4 = const(0x23)
LIS3DH_REG_OUT = const(0x28)

#LED Pins
ledPinRed = const(13); #D7
ledPinGreen = const(12); #D6
ledPinBlue = const(14); #D5
buzzPin = const(15);

duration = const(100);

mqttConnected = False;

"""
Define functions and global variables
"""
accel_reg_buf = bytearray(6);
accel_x = 0;
accel_y = 0;
accel_z = 0;
accelWarn = False;
sumdistance = 0;
sumaccel = 0;
distance = 0;

def combineBytes(data):
    value = data[0] << 8 | data[1]
    # Take the content of the combined bytes and shift to account for 12 bits 
    output = (value & 0xFFF) / 16.0
    # Twos complement
    if value & 0x1000:
        output -= 256.0
    return output

def readAccel():
    i2c.readfrom_mem_into(LIS3DH_ADDR, LIS3DH_REG_OUT, accel_reg_buf)
    accel_x = combineBytes(accel_reg_buf[0:2])
    accel_y = combineBytes(accel_reg_buf[2:4])
    accel_z = combineBytes(accel_reg_buf[4:6])

def setUpAccel():
    #Check LIS3DH connected

    # Enable all axes and 400hz: 01110111
    i2c.writeto_mem(LIS3DH_ADDR, LIS3DH_REG_CTRL1, b'\x77');
    
    # Set Block data update, high accuracy and +-4G range for LIS3DH: 10011000
    i2c.writeto_mem(LIS3DH_ADDR, LIS3DH_REG_CTRL4, b'\x98');
    
    return True;

def readDistance():
    try:
        distance = 0.2*adc.read(0);
    except:
        print('Error: cannot read adc - check i2c connection')
        distance = 5000
        
    if distance > 5000:
        distance = 5000;
    return distance

"""
Set-up outputs for LED and Buzzer
"""
class ledCtrl:
    def __init__(self):
        self.pinLED_Red = machine.Pin(ledPinRed, machine.Pin.OUT)
        self.pinLED_Green = machine.Pin(ledPinGreen, machine.Pin.OUT)
        self.pinLED_Blue = machine.Pin(ledPinBlue, machine.Pin.OUT)
        self.blinkTimer = machine.Timer(-1)
    def setOutput(self, pin, on):
        if on:
            pin.low();
        else:
            pin.high();
    def setAll(self, r,g,b):
        self.setOutput(pin=self.pinLED_Red, on=r);
        self.setOutput(pin=self.pinLED_Green, on=g);
        self.setOutput(pin=self.pinLED_Blue, on=b);
    def off(self):
        self.setAll(r=False, g=False, b=False)
    def red(self):
        self.setAll(r=True, g=False, b=False)
    def yellow(self):
        self.setAll(r=True, g=True, b=False)
    def green(self):
        self.setAll(r=False, g=True, b=False)
    def purple(self):
        self.setAll(r=True, g=False, b=True)
    def cyan(self):
        self.setAll(r=False, g=True, b=True)
    def toggle(self):
        self.pinLED_Red.value(not self.pinLED_Red.value());
        self.pinLED_Green.value(not self.pinLED_Green.value());
        self.pinLED_Blue.value(not self.pinLED_Blue.value());
    def setBlink(p):
        if p == 0:
            self.blinkTimer.deinit();
        else:
            self.blinkTimer.init(period=p, mode=Timer.PERIODIC, callback=lambda t:self.toggle())

class buzzCtrl:
    def __init__(self):
        self.pin_Buzz = machine.PWM(machine.Pin(buzzPin, machine.Pin.OUT))
        self.pin_Buzz.duty(0)
        self.pin_Buzz.freq(0)
        # Duration controlled in main loop
        # self.durationTimer = machine.Timer(-1)
    def off(self):
        self.pin_Buzz.freq(0)
        self.pin_Buzz.duty(0)
    def setFrequency(self,frequency):
        self.pin_Buzz.freq(frequency)
        self.pin_Buzz.duty(600)
    def setDuration(self, duration, tone):
        self.setFrequency(frequency=tone);
        # Duration controlled in main loop
        # self.durationTimer.init(period=duration, mode=Timer.PERIODIC, callback=lambda t: self.off());
    def high(self):
        self.setDuration(duration, 1000);
    def low(self):
        self.setDuration(duration, 600);    
        

led = ledCtrl();
led.purple();
buzz = buzzCtrl();



"""
Set up I2C
"""
i2c = machine.I2C(scl=machine.Pin(5), sda=machine.Pin(4), freq=100000)
print('Devices attaches:');
print(i2c.scan())

#Set up ADC
print('Setting up ADC')
adc = ads1x15.ADS1115(i2c)
adc.gain = 1; #ADS1015_REG_CONFIG_PGA_4_096V

#Set up LIS3DH
print('Setting up LIS3DH')
setUpAccel();


"""
Set up Network
"""
sta_if = network.WLAN(network.STA_IF); sta_if.active(True)
sta_if.scan()                             # Scan for available access points
sta_if.connect(wifi_ssid, wifi_passw) # Connect to an AP
# Check for successful connection
connectCount = 0;
while not sta_if.isconnected() and connectCount < 1000: # wait to connect
    connectCount += 1;

# Turn off ESP8266's AP
ap_if = network.WLAN(network.AP_IF)
ap_if.active(False)

"""
Set-up MQTT
"""
led.cyan();
client = MQTTClient(machine.unique_id(),mqtt_server)
try:
    client.connect()
    client.publish(mqtt_topic,bytes(data,'utf-8'))
    mqttConnected = True;
except:
    print('cannot connect to MQTT')
    
"""
Set-up Time
"""
last_mqtt = time.ticks_ms();
samples = 0;
toggle = False;

"""
Main processing loop
"""
while True:
    
    toggle = not toggle;
    samples += 1;

    # read data from sensor into variables
    distance = readDistance();
    readAccel();
    
    # Sum distance for avg distance calculation
    sumdistance += distance;
    sumaccel += accel_x;

    # Cycle turning detection
    accelWarn = abs(accel_x) > accelx_threshold;

    #Reset buzzer
    buzz.off()
    
    if distance < 500:
        led.red();
        if accelWarn:
            led.setBlink(200);
            if toggle:
                buzz.high()
            else:
                buzz.low()
        else:
            led.setBlink(300);
            buzz.low()
            
    elif distance < 1000:
        led.red();
        if accelWarn:
            led.setBlink(200);
            if toggle:
                buzz.high()
            else:
                buzz.low()
        else:            
            led.setBlink(500);
    elif distance < 1500:
        led.red();
        
        if accelWarn:
            led.setBlink(400);
            if toggle:
                buzz.high()
            else:
                buzz.low()
        else:
            led.setBlink(0)
    elif distance < 2000:
        led.amber();
        if accelWarn:
            led.setBlink(400);
            if toggle:
                buzz.high()
            else:
                buzz.low()
        else:
            led.setBlink(0)
    elif distance < 2500:
        led.amber();
        if accelWarn:
            led.setBlink(400);
        else:
            led.setBlink(0)
    else:
        led.green();
        led.setBlink(0)

    #Crash detection
    if abs(accel_z) > accelz_threshold:
        led.purple();
        led.setBlink(500);
        client.publish(mqtt_topic, b"crash", True);
        timer.sleep_ms(5000);
    
    # Report data collected to MQTT regularly    
    timeDiff = time.ticks_diff(last_mqtt, time.ticks_ms());
    if timeDiff > 500:
        # Find the delta of distance and divide by 100 for m and 1000 and ms->sec
        velocity = (lastAvgDist - sumdistance/sampletaken)/timeDiff/100000;
        lastAvgDist = sumdistance/sampletaken;
        # encode all data as a JSON
        payload = ujson.dumps({"accel" : (sumaccel/samples), "warn" : accelWarn, "distance" : (lastAvgDist/100), "Vapproach" : min(0,velocity)})
        # push to MQTT
        if mqttConnected:
            client.publish(mqtt_topic, payload)
        # Reset accumulator
        sumdistance = 0;
        sumaccele = 0;
        samples = 0
        last_mqtt = time.ticks_ms()
        
    timer.sleep_ms(150);
   
