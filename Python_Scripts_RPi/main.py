import time, sys
import seeed_dht
import RPi.GPIO as GPIO
import paho.mqtt.client as mqtt
from time import sleep
import smbus
from Adafruit_I2C import Adafruit_I2C
from smbus import SMBus
import Adafruit_ADS1x15
import RPi.GPIO as gpio
import cv2
import json
from ircodec.command import CommandSet

# gpio.setmode(gpio.BCM)

GPIO.setmode(GPIO.BCM)
gpio.setmode(gpio.BCM)

# Fan Index = 0
# LED Index = 1
commands = [1, 1, 1]
prev_states = [1, 1, 1]
states = [1, 1, 1]
current_states = [1, 1, 1]
NumPeople = [1]

class Ultrasonic:
    #set GPIO Pins
    GPIO_TRIGGER = 23
    GPIO_ECHO = 24

    def setup(self):
        # GPIO.cleanup()
        #GPIO Mode (BOARD / BCM)
        # GPIO.setmode(GPIO.BCM)
        # GPIO.setmode(GPIO.BOARD)

        #set GPIO direction (IN / OUT)
        GPIO.setup(self.GPIO_TRIGGER, GPIO.OUT)
        GPIO.setup(self.GPIO_ECHO, GPIO.IN)

    def distance(self):
        # set Trigger to HIGH
        GPIO.output(self.GPIO_TRIGGER, True)

        # set Trigger after 0.01ms to LOW
        time.sleep(0.00001)
        GPIO.output(self.GPIO_TRIGGER, False)

        StartTime = time.time()
        StopTime = time.time()
        # save StartTime
        while GPIO.input(self.GPIO_ECHO) == 0:
            StartTime = time.time()

        # save time of arrival
        while GPIO.input(self.GPIO_ECHO) == 1:
            StopTime = time.time()


        # time difference between start and arrival
        TimeElapsed = StopTime - StartTime
        # multiply with the sonic speed (34300 cm/s)
        # and divide by 2, because there and back
        distance = (TimeElapsed * 34300) / 2

        return distance

    # if __name__ == '__main__':
        # client = mqtt.Client()
        # client.connect("172.16.116.133",1883,60)
    def main(self):
        try:
            # while True:
            dist = self.distance()
            # print ("Measured Distance = %.1f cm" % dist, file=open("./../logfiles/Ultrasonic.txt", "a"))
            # print ("Measured Distance = %.1f cm" % dist)
            var = "Measured Distance = %.1f cm" % dist
            print(var)
            client.publish("topic/ultrasonic", var)
            return dist
            # time.sleep(1)

            # Reset by pressing CTRL + C
        except KeyboardInterrupt:
            print("Measurement stopped by User")
            var = "Measurement stopped by User"
            client.publish("topic/ultrasonic", var)
            # GPIO.cleanup()

class Temp:

    def main(self):
        # for DHT11/DHT22
        sensor = seeed_dht.DHT("11", 12)
        # for DHT10
        # sensor = seeed_dht.DHT("10")

        # while True:
        humi, temp = sensor.read()

        #for x in range(1,50):
        var = ""
        if not humi is None:
            var = "DHT{0}, humidity {1:.1f}%, temperature {2:.1f}*".format(sensor.dht_type, humi, temp)
            # print('DHT{0}, humidity {1:.1f}%, temperature {2:.1f}*'.format(sensor.dht_type, humi, temp), file=open("./../logfiles/Temp_Hum.txt", "a"))
            print('DHT{0}, humidity {1:.1f}%, temperature {2:.1f}*'.format(sensor.dht_type, humi, temp))
        else:
            # print('DHT{0}, humidity & temperature: {1}'.format(sensor.dht_type, temp), file=open("./../logfiles/Temp_Hum.txt", "a"))
            var = "DHT{0}, humidity & temperature: {1}'.format(sensor.dht_type, temp)"
            print('DHT{0}, humidity & temperature: {1}'.format(sensor.dht_type, temp))
        # print(var)
        client.publish("topic/temp", var)
        return temp
        # time.sleep(1)


# Light Sensor
TSL2561_Control = 0x80
TSL2561_Timing = 0x81
TSL2561_Interrupt = 0x86
TSL2561_Channel0L = 0x8C
TSL2561_Channel0H = 0x8D
TSL2561_Channel1L = 0x8E
TSL2561_Channel1H = 0x8F

TSL2561_Address = 0x29 #device address

LUX_SCALE = 14 # scale by 2^14
RATIO_SCALE = 9 # scale ratio by 2^9
CH_SCALE = 10 # scale channel values by 2^10
CHSCALE_TINT0 = 0x7517 # 322/11 * 2^CH_SCALE
CHSCALE_TINT1 = 0x0fe7 # 322/81 * 2^CH_SCALE

K1T = 0x0040 # 0.125 * 2^RATIO_SCALE
B1T = 0x01f2 # 0.0304 * 2^LUX_SCALE
M1T = 0x01be # 0.0272 * 2^LUX_SCALE
K2T = 0x0080 # 0.250 * 2^RATIO_SCA
B2T = 0x0214 # 0.0325 * 2^LUX_SCALE
M2T = 0x02d1 # 0.0440 * 2^LUX_SCALE
K3T = 0x00c0 # 0.375 * 2^RATIO_SCALE
B3T = 0x023f # 0.0351 * 2^LUX_SCALE
M3T = 0x037b # 0.0544 * 2^LUX_SCALE
K4T = 0x0100 # 0.50 * 2^RATIO_SCALE
B4T = 0x0270 # 0.0381 * 2^LUX_SCALE
M4T = 0x03fe # 0.0624 * 2^LUX_SCALE
K5T = 0x0138 # 0.61 * 2^RATIO_SCALE
B5T = 0x016f # 0.0224 * 2^LUX_SCALE
M5T = 0x01fc # 0.0310 * 2^LUX_SCALE
K6T = 0x019a # 0.80 * 2^RATIO_SCALE
B6T = 0x00d2 # 0.0128 * 2^LUX_SCALE
M6T = 0x00fb # 0.0153 * 2^LUX_SCALE
K7T = 0x029a # 1.3 * 2^RATIO_SCALE
B7T = 0x0018 # 0.00146 * 2^LUX_SCALE
M7T = 0x0012 # 0.00112 * 2^LUX_SCALE
K8T = 0x029a # 1.3 * 2^RATIO_SCALE
B8T = 0x0000 # 0.000 * 2^LUX_SCALE
M8T = 0x0000 # 0.000 * 2^LUX_SCALE



K1C = 0x0043 # 0.130 * 2^RATIO_SCALE
B1C = 0x0204 # 0.0315 * 2^LUX_SCALE
M1C = 0x01ad # 0.0262 * 2^LUX_SCALE
K2C = 0x0085 # 0.260 * 2^RATIO_SCALE
B2C = 0x0228 # 0.0337 * 2^LUX_SCALE
M2C = 0x02c1 # 0.0430 * 2^LUX_SCALE
K3C = 0x00c8 # 0.390 * 2^RATIO_SCALE
B3C = 0x0253 # 0.0363 * 2^LUX_SCALE
M3C = 0x0363 # 0.0529 * 2^LUX_SCALE
K4C = 0x010a # 0.520 * 2^RATIO_SCALE
B4C = 0x0282 # 0.0392 * 2^LUX_SCALE
M4C = 0x03df # 0.0605 * 2^LUX_SCALE
K5C = 0x014d # 0.65 * 2^RATIO_SCALE
B5C = 0x0177 # 0.0229 * 2^LUX_SCALE
M5C = 0x01dd # 0.0291 * 2^LUX_SCALE
K6C = 0x019a # 0.80 * 2^RATIO_SCALE
B6C = 0x0101 # 0.0157 * 2^LUX_SCALE
M6C = 0x0127 # 0.0180 * 2^LUX_SCALE
K7C = 0x029a # 1.3 * 2^RATIO_SCALE
B7C = 0x0037 # 0.00338 * 2^LUX_SCALE
M7C = 0x002b # 0.00260 * 2^LUX_SCALE
K8C = 0x029a # 1.3 * 2^RATIO_SCALE
B8C = 0x0000 # 0.000 * 2^LUX_SCALE
M8C = 0x0000 # 0.000 * 2^LUX_SCALE

# bus parameters
rev = GPIO.RPI_REVISION
if rev == 2 or rev == 3:
	bus = smbus.SMBus(1)
else:
	bus = smbus.SMBus(0)
i2c = Adafruit_I2C(TSL2561_Address)

debug = False
cooldown_time = 0.005 # measured in seconds
packageType = 0 # 0=T package, 1=CS package
gain = 0        # current gain: 0=1x, 1=16x [dynamically selected]
gain_m = 1      # current gain, as multiplier
timing = 2      # current integration time: 0=13.7ms, 1=101ms, 2=402ms [dynamically selected]
timing_ms = 0   # current integration time, in ms
channel0 = 0    # raw current value of visible+ir sensor
channel1 = 0    # raw current value of ir sensor
schannel0 = 0   # normalized current value of visible+ir sensor
schannel1 = 0   # normalized current value of ir sensor


def readRegister(address):
	try:
		byteval = i2c.readU8(address)

		sleep(cooldown_time)
		if (debug):
			print("TSL2561.readRegister: returned 0x%02X from reg 0x%02X" % (byteval, address))
		return byteval
	except IOError:
		print("TSL2561.readRegister: error reading byte from reg 0x%02X" % address)
		return -1


def writeRegister(address, val):
	try:
		i2c.write8(address, val)

		sleep(cooldown_time)
		if (debug):
			print("TSL2561.writeRegister: wrote 0x%02X to reg 0x%02X" % (val, address))
	except IOError:

		sleep(cooldown_time)
		print("TSL2561.writeRegister: error writing byte to reg 0x%02X" % address)
		return -1

def powerUp():
	writeRegister(TSL2561_Control, 0x03)

def powerDown():
	writeRegister(TSL2561_Control, 0x00)

def setTintAndGain():
	global gain_m, timing_ms

	if gain == 0:
		gain_m = 1
	else:
		gain_m = 16

	if timing == 0:
		timing_ms = 13.7
	elif timing == 1:
		timing_ms = 101
	else:
		timing_ms = 402
	writeRegister(TSL2561_Timing, timing | gain << 4)

def readLux():
	sleep(float(timing_ms + 1) / 1000)

	ch0_low  = readRegister(TSL2561_Channel0L)
	ch0_high = readRegister(TSL2561_Channel0H)
	ch1_low  = readRegister(TSL2561_Channel1L)
	ch1_high = readRegister(TSL2561_Channel1H)

	global channel0, channel1
	channel0 = (ch0_high<<8) | ch0_low
	channel1 = (ch1_high<<8) | ch1_low

	sleep(cooldown_time)
	if debug:
		print("TSL2561.readVisibleLux: channel 0 = %i, channel 1 = %i [gain=%ix, timing=%ims]" % (channel0, channel1, gain_m, timing_ms))

def readVisibleLux():
	global timing, gain

	powerUp()
	readLux()

	if channel0 < 500 and timing == 0:
		timing = 1
		sleep(cooldown_time)
		if debug:
			print("TSL2561.readVisibleLux: too dark. Increasing integration time from 13.7ms to 101ms")
		setTintAndGain()
		readLux()

	if channel0 < 500 and timing == 1:
		timing = 2
		sleep(cooldown_time)
		if debug:
			print("TSL2561.readVisibleLux: too dark. Increasing integration time from 101ms to 402ms")
		setTintAndGain()
		readLux()

	if channel0 < 500 and timing == 2 and gain == 0:
		gain = 1
		sleep(cooldown_time)
		if debug:
			print("TSL2561.readVisibleLux: too dark. Setting high gain")
		setTintAndGain()
		readLux()

	if (channel0 > 20000 or channel1 > 20000) and timing == 2 and gain == 1:
		gain = 0
		sleep(cooldown_time)
		if debug:
			print("TSL2561.readVisibleLux: enough light. Setting low gain")
		setTintAndGain()
		readLux()

	if (channel0 > 20000 or channel1 > 20000) and timing == 2:
		timing = 1
		sleep(cooldown_time)
		if debug:
			print("TSL2561.readVisibleLux: enough light. Reducing integration time from 402ms to 101ms")
		setTintAndGain()
		readLux()

	if (channel0 > 10000 or channel1 > 10000) and timing == 1:
		timing = 0
		sleep(cooldown_time)
		if debug:
			print("TSL2561.readVisibleLux: enough light. Reducing integration time from 101ms to 13.7ms")
		setTintAndGain()
		readLux()

	powerDown()

	if (timing == 0 and (channel0 > 5000 or channel1 > 5000)) or (timing == 1 and (channel0 > 37000 or channel1 > 37000)) or (timing == 2 and (channel0 > 65000 or channel1 > 65000)):
		# overflow
		return -1

	return calculateLux(channel0, channel1)

def calculateLux(ch0, ch1):
	chScale = 0
	if timing == 0:   # 13.7 msec
		chScale = CHSCALE_TINT0
	elif timing == 1: # 101 msec
		chScale = CHSCALE_TINT1;
	else:           # assume no scaling
		chScale = (1 << CH_SCALE)

	if gain == 0:
		chScale = chScale << 4 # scale 1X to 16X

	# scale the channel values
	global schannel0, schannel1
	schannel0 = (ch0 * chScale) >> CH_SCALE
	schannel1 = (ch1 * chScale) >> CH_SCALE

	ratio = 0
	if schannel0 != 0:
		ratio = (schannel1 << (RATIO_SCALE+1)) / schannel0
	ratio = (ratio + 1) // 2

	if packageType == 0: # T package
		if ((ratio >= 0) and (ratio <= K1T)):
			b=B1T; m=M1T;
		elif (ratio <= K2T):
			b=B2T; m=M2T;
		elif (ratio <= K3T):
			b=B3T; m=M3T;
		elif (ratio <= K4T):
			b=B4T; m=M4T;
		elif (ratio <= K5T):
			b=B5T; m=M5T;
		elif (ratio <= K6T):
			b=B6T; m=M6T;
		elif (ratio <= K7T):
			b=B7T; m=M7T;
		elif (ratio > K8T):
			b=B8T; m=M8T;
	elif packageType == 1: # CS package
		if ((ratio >= 0) and (ratio <= K1C)):
			b=B1C; m=M1C;
		elif (ratio <= K2C):
			b=B2C; m=M2C;
		elif (ratio <= K3C):
			b=B3C; m=M3C;
		elif (ratio <= K4C):
			b=B4C; m=M4C;
		elif (ratio <= K5C):
			b=B5C; m=M5C;
		elif (ratio <= K6C):
			b=B6C; m=M6C;
		elif (ratio <= K7C):
			b=B7C; m=M7C;

	temp = ((schannel0*b)-(schannel1*m))
	if temp < 0:
		temp = 0;
	temp += (1<<(LUX_SCALE-1))
	# strip off fractional portion
	lux = temp>>LUX_SCALE
	sleep(cooldown_time)
	if debug:
		print("TSL2561.calculateLux: %i" % lux)

	return lux

def init():
	powerUp()
	setTintAndGain()
	writeRegister(TSL2561_Interrupt, 0x00)
	powerDown()

def light():
    init()
    Light = readVisibleLux()
    var = "Lux: %i [Vis+IR=%i, IR=%i @ Gain=%ix, Timing=%.1fms]" % (Light, channel0, channel1, gain_m, timing_ms)
    print(var)
    client.publish("topic/light", var)
    return Light


    # sleep(1)

# if __name__ == "__main__":
#         main()




__all__ = ["GroveAirQualitySensor"]
buzzer = 22

def air_setup():
    # GPIO.cleanup()
    # print(GPIO.getmode())
    # GPIO.setmode(GPIO.BOARD)
    # gpio.setmode(gpio.BCM)
    gpio.setup(buzzer,gpio.OUT)

class GroveAirQualitySensor(object):
    '''
    Grove Air Quality Sensor class

    Args:
        pin(int): number of analog pin/channel the sensor connected.
    '''
    def __init__(self, channel):
        self.channel = channel
        print("Channel: ", self.channel)
        # self.adc = ADC()
        self.adc = Adafruit_ADS1x15.ADS1115()

    @property
    def value(self):
        '''
        Get the air quality strength value, badest value is 100.0%.

        Returns:
            (int): ratio, 0(0.0%) - 1000(100.0%)
        '''
        return self.adc.read_adc(self.channel)

Grove = GroveAirQualitySensor


def air():
    # from grove.helper import SlotHelper
    # sh = SlotHelper(SlotHelper.ADC)
    # pin = sh.argv2pin()
    pin = 0

    sensor = GroveAirQualitySensor(pin)

    # print('Detecting ...')
    # while True:
    try:
        value = sensor.value
        if value > 2000:
            gpio.output(buzzer,gpio.HIGH)
            var = "{}, High Pollution.".format(value)
            print(var)
            client.publish("topic/notif", "ALERT: Air pollution is very high. Keep your doors and windows open.")
            client.publish("topic/air", var)
        else:
            var = "{}, Air Quality OK.".format(value)
            print(var)
            client.publish("topic/air", var)
            gpio.output(buzzer,gpio.LOW)
        # time.sleep(1)
    except KeyboardInterrupt:
        gpio.output(buzzer,gpio.LOW)
        # break
    except IOError:
        print ("Error")
    # pass

# if __name__ == '__main__':
#     main()

broker_address="172.16.116.133"

print("Step 0")
capture = cv2.VideoCapture(-1)
# print("Step 1")
# cam_r2, cam_img = capture.read()
# print("Step 2")
# cam_img_list = cam_img.tolist()
# print("Step 3")
# cam_json = json.dumps(cam_img_list)
# print("Step 4")



def on_connect(client, userdata, flags, rc):
	print("Connected switch client with result code "+str(rc))
	client.subscribe("topic/switch")

def on_message(client, userdata, msg):
    code = msg.payload.decode()
    print(code)
    commands[int(code[1])] = int(code[0])
    print("Command received:", commands)

def cam_on_connect(client, userdata, flags, rc):
	print("Connected cam client with result code "+str(rc))
	client.subscribe("topic/cam_result")

def cam_on_message(client, userdata, msg):
    code = msg.payload.decode()
    var = str(code)
    print(var)
    NumPeople[0] = int(var)
    client.publish("topic/webcam", var)

ultrasonic = Ultrasonic()
temp = Temp()
# light = Light()


client = mqtt.Client()
client.connect("172.16.116.133",1883,60)

cam_client = mqtt.Client()
cam_client.connect("172.16.116.133",1883,60)

FAN_PIN = 27
WAIT_TIME = 1
PWM_FREQ = 100

GPIO.setup(FAN_PIN, GPIO.OUT, initial=GPIO.HIGH)

fan=GPIO.PWM(FAN_PIN,PWM_FREQ)
fan.start(0);
i = 0

hyst = 1
tempSteps = [50, 70]
speedSteps = [0, 100]
cpuTempOld = 0

client.on_connect = on_connect
client.on_message = on_message

cam_client.on_connect = cam_on_connect
cam_client.on_message = cam_on_message

led_pin = 17
GPIO.setup(led_pin, GPIO.OUT)
pwm = GPIO.PWM(led_pin, 100)
pwm.start(0)

ir_controller = CommandSet.load('ac.json')
# client.loop_forever()

PrevState = 0
PrevTemp = 0
# ir_controller.emit("power_on")

AC_Temp = 20
AC_Required = 0

while True:
    try:
        Temp = temp.main()
        fanSpeed = 0

        if Temp == 0:
            fanSpeed = 60
            states[0] = 1
            states[2] = 0
        elif Temp < 20:
            states[0] = 0   #Fan
            states[2] = 0   #AC
        elif Temp >= 20 and Temp < 27:
            fanSpeed = 60
            states[0] = 1
            states[2] = 0
        elif Temp >= 27 and Temp < 30:
            fanSpeed = 70
            states[0] = 1
            states[2] = 1
            AC_Required = 25
        elif Temp >= 30 and Temp < 35:
            states[0] = 1
            fanSpeed = 80
            states[2] = 1
            AC_Required = 20
        elif Temp >= 35 and Temp < 40:
            states[0] = 1
            fanSpeed = 90
            states[2] = 1
            AC_Required = 16
        else:
            states[0] = 1
            fanSpeed = 100
            states[2] = 1
            AC_Required = 16

        Light = light()

        LEDBrightness = 0

        states[1] = 1
        if Light <= 100:
            # states[1] = 1
            LEDBrightness = 100
        elif Light <= 200:
            # states[1] = 1
            LEDBrightness = 90
        elif Light <= 300:
            # states[1] = 1
            LEDBrightness = 80
        elif Light <= 400:
            # states[1] = 1
            LEDBrightness = 70
        elif Light <= 500:
            # states[1] = 1
            LEDBrightness = 60
        elif Light <= 600:
            LEDBrightness = 50
        elif Light <= 700:
            LEDBrightness = 40
        elif Light <= 800:
            LEDBrightness = 30
        elif Light <= 900:
            LEDBrightness = 20
        elif Light <= 1000:
            LEDBrightness = 10
        else:
            states[1] = 0
            LEDBrightness = 0

        air_setup()
        air()

        ultrasonic.setup()
        ultra_dist = ultrasonic.main()


        print("Step 1")
        cam_r2, cam_img = capture.read()
        print("Step 2")
        cam_img_list = cam_img.tolist()
        print("Step 3")
        cam_json = json.dumps(cam_img_list)
        print("Step 4")
        client.publish("topic/send_photo", cam_json)
        print("Step 5")

        client.loop(5)
        cam_client.loop(2)

        print("NumPeople:", NumPeople[0])
        for i in range(0, 3):
            if commands[i] == 0:
                current_states[i] = 0
            elif NumPeople[0] == 0:
                current_states[i] = 0
            else:
                current_states[i] = states[i]

        print("Current mobile commands:", commands)

        if NumPeople[0] == 0:
            client.publish("topic/notif", "There is nobody in the room, so power has been shut down.")

        if NumPeople[0] >= 4 and NumPeople[0] < 8:
            fanSpeed = fanSpeed * 1.2
            fanSpeed = int(fanSpeed)
        elif NumPeople[0] >= 8:
            fanSpeed = fanSpeed * 1.5
            fanSpeed = int(fanSpeed)
        fanSpeed = min(fanSpeed, 100)

        fanSpeed *= current_states[0]
        print("Fan Speed: ", fanSpeed)
        fan.ChangeDutyCycle(fanSpeed)

        if PrevState != current_states[2]:
            if current_states[2] == 0:
                client.publish("topic/notif", "The AC has been switched off.")
                ir_controller.emit("power_off")
            else:
                client.publish("topic/notif", "The AC has been switched on.")
                ir_controller.emit("power_on")
                print("Sending IR for power on")
                AC_Temp = 20
            PrevState = current_states[2]

        if NumPeople[0] >= 4 and NumPeople[0] < 8:
            AC_Required = AC_Required / 1.2
            AC_Required = int(AC_Required)
            AC_Required = max(AC_Required, 16)
        elif NumPeople[0] >= 8:
            AC_Required = AC_Required / 1.5
            AC_Required = int(AC_Required)
            AC_Required = max(AC_Required, 16)

        if current_states[2]:
            while AC_Required > AC_Temp:
                ir_controller.emit("temp_up")
                AC_Temp += 1
                # sleep()
            while AC_Temp > AC_Required:
                ir_controller.emit("temp_down")
                AC_Temp -= 1
                # sleep()

        if current_states[2] and ultra_dist > 30:
            client.publish("topic/notif", "Inefficient cooling due to open door!")

        LEDBrightness *= current_states[1]
        pwm.ChangeDutyCycle(LEDBrightness)

        states_send = ""
        for i in range(0, 3):
            states_send += str(current_states[i])

        client.publish("topic/read", states_send)
        print("Current state:", current_states)

    except:
        ir_controller.emit("power_off")
        GPIO.cleanup()
        gpio.cleanup()
        print("Error")
        break

    # Temp = temp.main()
    # fanSpeed = 0
    #
    # if Temp < 20:
    #     states[0] = 0   #Fan
    #     states[2] = 0   #AC
    # elif Temp >= 20 and Temp < 25:
    #     fanSpeed = 60
    #     states[0] = 1
    #     states[2] = 0
    # elif Temp >= 25 and Temp < 30:
    #     fanSpeed = 70
    #     states[0] = 1
    #     states[2] = 0
    # elif Temp >= 30 and Temp < 35:
    #     states[0] = 1
    #     fanSpeed = 80
    #     states[2] = 1
    # elif Temp >= 35 and Temp < 40:
    #     states[0] = 1
    #     fanSpeed = 90
    #     states[2] = 1
    # else:
    #     states[0] = 1
    #     fanSpeed = 100
    #     states[2] = 1
    #
    # Light = light()
    #
    # LEDBrightness = 0
    #
    # states[1] = 1
    # if Light <= 100:
    #     # states[1] = 1
    #     LEDBrightness = 100
    # elif Light <= 200:
    #     # states[1] = 1
    #     LEDBrightness = 90
    # elif Light <= 300:
    #     # states[1] = 1
    #     LEDBrightness = 80
    # elif Light <= 400:
    #     # states[1] = 1
    #     LEDBrightness = 70
    # elif Light <= 500:
    #     # states[1] = 1
    #     LEDBrightness = 60
    # elif Light <= 600:
    #     LEDBrightness = 50
    # elif Light <= 700:
    #     LEDBrightness = 40
    # elif Light <= 800:
    #     LEDBrightness = 30
    # elif Light <= 900:
    #     LEDBrightness = 20
    # elif Light <= 1000:
    #     LEDBrightness = 10
    # else:
    #     states[1] = 0
    #     LEDBrightness = 0
    #
    # air_setup()
    # air()
    #
    # ultrasonic.setup()
    # ultrasonic.main()
    #
    #
    # print("Step 1")
    # cam_r2, cam_img = capture.read()
    # print("Step 2")
    # cam_img_list = cam_img.tolist()
    # print("Step 3")
    # cam_json = json.dumps(cam_img_list)
    # print("Step 4")
    # client.publish("topic/send_photo", cam_json)
    # print("Step 5")
    #
    # client.loop(5)
    # cam_client.loop(2)
    #
    # print("NumPeople:", NumPeople[0])
    # for i in range(0, 3):
    #     if commands[i] == 0:
    #         current_states[i] = 0
    #     elif NumPeople[0] == 0:
    #         current_states[i] = 0
    #     else:
    #         current_states[i] = states[i]
    #
    # fanSpeed *= current_states[0]
    # print("Fan Speed: ", fanSpeed)
    # fan.ChangeDutyCycle(fanSpeed)
    #
    # if PrevState != current_states[2]:
    #     if current_states[2] == 0:
    #         ir_controller.emit("power_off")
    #     else:
    #         ir_controller.emit("power_on")
    #     PrevState = current_states[2]
    #
    #
    # LEDBrightness *= current_states[1]
    # pwm.ChangeDutyCycle(LEDBrightness)
    #
    # states_send = ""
    # for i in range(0, 3):
    #     states_send += str(current_states[i])
    #
    # client.publish("topic/read", states_send)
    # print("Current state:", current_states)

client.disconnect()
