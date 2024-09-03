
try:
	import busio
	from board import SCL_1, SDA_1
	from adafruit_pca9685 import PCA9685
	from adafruit_motor import servo
except:
	print("\nWARNING: Unable to import servo motor libraries. Continuing without servo motors.")
	SCL_1 = 0
	SDA_1 = 0
	class busio:
		@staticmethod
		def I2C(scl, sda):
			return None
	class PCA9685:
		def __init__(self, i2c, address):
			self.i2c = i2c
			self.address = address
			self.frequency = 0
			self.channels = list(range(16))
	class servo:
		def __init__(self, channel, min_pulse, max_pulse, actuation_range):
			self.channel = channel
			self.min_pulse = min_pulse
			self.max_pulse = max_pulse
			self.actuation_range = actuation_range
			self.angle = 0
		@staticmethod
		def Servo(channel, min_pulse = 0, max_pulse = 0, actuation_range = 0):
			return servo(channel, min_pulse, max_pulse, actuation_range)

class ServoController:
	def __init__(self, address = 0x40):
		self.i2c = busio.I2C(SCL_1, SDA_1)
		self.pca = PCA9685(self.i2c, address = address)
		self.pca.frequency = 333
		self.servoList = dict()

	def addServo(self, channel, min_pulse = 500, max_pulse = 2500, actuation_range = 180):
		s = servo.Servo(self.pca.channels[channel], min_pulse = min_pulse, max_pulse = max_pulse, actuation_range = actuation_range)
		self.servoList[channel] = s
		return s
	
	def getServo(self, channel):
		return self.servoList[channel]
