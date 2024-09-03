
from time import time

try:
	from controller import Robot as Webot
except:
	print("\nWARNING: Unable to import Webots library. Continuing without Webots functionality.")
	class Webot:
		def getDevice(self, jointName):
			return None
		def getBasicTimeStep(self):
			return 0
		def step(self, timeStep):
			return False

class WebotsServoController:
	def __init__(self):
		self.webot = Webot()
		self.simStep = int(self.webot.getBasicTimeStep())
		self.reset()

	def reset(self):
		self.simTime = int(0)
		self.startTime = time()

	def update(self):
		self.simTime = self.simTime + self.simStep if self.simStep > 0 else round(1000 * (time() - self.startTime))
		return self.webot.step(self.simStep) == -1

	def addServo(self, jointName):
		s = self.webot.getDevice(jointName)
		if s is not None:
			s.setPosition(0)
		return s
