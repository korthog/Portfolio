
from math import cos, pi

class GaitTrajectory:
	@staticmethod
	def walk(t, i):
		t = (t + i) % 4
		if t < 1:
			x = ((((-80*t + 280)*t - 336)*t + 140)*t*t*t - 1)*t / 3 - 0.5
			y = 0.5 + 0.5 * cos(2*pi*(t - 0.5))
		else:
			x = (1 - t) / 3 + 0.5
			y = 0
		return x, y

	@staticmethod
	def trot(t, i):
		t = (t + (0, 1, 1, 0)[i]) % 2
		if t < 1:
			x = ((((-40*t + 140)*t - 168)*t + 70)*t*t*t - 1)*t - 0.5
			y = 0.5 + 0.5 * cos(2*pi*(t - 0.5))
		else:
			x = 1.5 - t
			y = 0
		return x, y

class QuadrupedGait:
	def __init__(self,
		trajectory = GaitTrajectory.walk,
		descriptionText = "Not Specified",
		stepFrequency = 4,
		minStepHeight = 0.2, maxStepHeight = 3,
		minBodyHeight = 1.5, maxBodyHeight = 7,
		xStride = 3, yStride = 3,
		spin = 0.1, pitch = 0.2,
		xSway = 0, ySway = 0,
		xStance = 0, yStance = 0):

		self.trajectory = trajectory
		self.descriptionText = descriptionText
		self.stepFrequency = stepFrequency
		self.minStepHeight = minStepHeight
		self.maxStepHeight = maxStepHeight
		self.minBodyHeight = minBodyHeight
		self.maxBodyHeight = maxBodyHeight
		self.xStride = xStride
		self.yStride = yStride
		self.spin = spin
		self.pitch = pitch
		self.xSway = xSway
		self.ySway = ySway
		self.xStance = xStance
		self.yStance = yStance
