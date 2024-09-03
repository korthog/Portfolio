
class OrigamiMotor:
	def __init__(self, origamiController, channel):
		self.origamiController = origamiController
		self.channel = channel
		self.offset = self.origamiController.getMotorPosition(channel)
	
	def setEffort(self, value):
		self.origamiController.setMotorPwm(self.channel, value)
	
	def getPosition(self):
		return self.origamiController.getMotorPosition(self.channel) - self.offset

	def zero(self):
		self.offset = self.origamiController.getMotorPosition(self.channel)
