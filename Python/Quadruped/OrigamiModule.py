
class OrigamiModule:
	def __init__(self, motor1, motor2, motor3):
		self.motors = [motor1, motor2, motor3]
		self.errorIntegral = [0] * len(self.motors)

	def runPid(self, pos, vel):
		kp = 2e-4
		ki = 1e-2
		kv = 1.95
		for i, motor in enumerate(self.motors):
			e = pos[i] - motor.getPosition()
			self.errorIntegral[i] = max(-1, min(1, self.errorIntegral[i] + ki * e))
			motor.setEffort(kv * (vel + kp * (e + self.errorIntegral[i])))

	def runMotors(self, effort):
		if not hasattr(effort, '__len__'):
			for motor in self.motors:
				motor.setEffort(effort)
		elif len(effort) == len(self.motors):
			for i, motor in enumerate(self.motors):
				motor.setEffort(effort[i])
		else:
			raise ValueError(f"Origami Motor Argument Length Error: effort must contain {len(self.motors)} elements.")
	
	def stop(self):
		for motor in self.motors:
			motor.setEffort(0)

	def zeroAll(self):
		for motor in self.motors:
			motor.zero()
	
	def getMotorPositions(self):
		return [motor.getPosition() for motor in self.motors]
