
from math import radians

class QuadrupedServo:
	def __init__(self, offset = 0, physicalServo = None, webotsServo = None):
		self.offset = offset
		self.physicalServo = physicalServo
		self.webotsServo = webotsServo
		self.angle = 0

	def setAngle(self, angle):
		self.angle = angle
		if self.physicalServo is not None:
			self.physicalServo.angle = max(0, min(180, angle + self.offset))
		if self.webotsServo is not None:
			self.webotsServo.setPosition(radians(angle))
