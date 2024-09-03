
from time import time, localtime, strftime
from math import sin, cos, pi
from QuadrupedGait import QuadrupedGait, GaitTrajectory
from QuadrupedLeg import QuadrupedLeg
from QuadrupedServo import QuadrupedServo
from ServoController import ServoController
from WebotsServoController import WebotsServoController
from OrigamiController import OrigamiController
from OrigamiModule import OrigamiModule
from OrigamiMotor import OrigamiMotor

class Quadruped:
	def __init__(self):
		self.ServoController = ServoController(address = 0x40)
		self.WebotsController = WebotsServoController()
		self.OrigamiController = OrigamiController()
		self.updatePeriod = 0
		self.updateRate = 0
		
		self.Gaits = [
			QuadrupedGait(descriptionText = "Fast Walk"),
			QuadrupedGait(descriptionText = "Fast Trot", trajectory = GaitTrajectory.trot),
			QuadrupedGait(descriptionText = "Slow Trot", trajectory = GaitTrajectory.trot, stepFrequency = 2),
			QuadrupedGait(descriptionText = "Slow Walk with Sway", maxBodyHeight = 6.5, stepFrequency = 0.6, xSway = 1, ySway = 2),
			QuadrupedGait(descriptionText = "Fast Walk with Wide Stance", maxBodyHeight = 6, xStance = 1, yStance = 1)]
		self.activeGait = int(0)

		self.Spine = OrigamiModule(
			OrigamiMotor(self.OrigamiController, channel = 5),
			OrigamiMotor(self.OrigamiController, channel = 7),
			OrigamiMotor(self.OrigamiController, channel = 6))

		FrontLeftLeg = QuadrupedLeg(
			legType = QuadrupedLeg.FRONT_LEFT,
			hip1 = QuadrupedServo(offset = 130, physicalServo = self.ServoController.addServo(4), webotsServo = self.WebotsController.addServo("FrontLeftHip1")),
			hip2 = QuadrupedServo(offset = 120, physicalServo = self.ServoController.addServo(5), webotsServo = self.WebotsController.addServo("FrontLeftHip2")),
			knee = QuadrupedServo(offset = 35, physicalServo = self.ServoController.addServo(6), webotsServo = self.WebotsController.addServo("FrontLeftKnee")))

		FrontRightLeg = QuadrupedLeg(
			legType = QuadrupedLeg.FRONT_RIGHT,
			hip1 = QuadrupedServo(offset = 20, physicalServo = self.ServoController.addServo(0), webotsServo = self.WebotsController.addServo("FrontRightHip1")),
			hip2 = QuadrupedServo(offset = 58, physicalServo = self.ServoController.addServo(2), webotsServo = self.WebotsController.addServo("FrontRightHip2")),
			knee = QuadrupedServo(offset = 147, physicalServo = self.ServoController.addServo(1), webotsServo = self.WebotsController.addServo("FrontRightKnee")))

		BackLeftLeg = QuadrupedLeg(
			legType = QuadrupedLeg.BACK_LEFT,
			hip1 = QuadrupedServo(offset = 10, physicalServo = self.ServoController.addServo(8), webotsServo = self.WebotsController.addServo("BackLeftHip1")),
			hip2 = QuadrupedServo(offset = 54, physicalServo = self.ServoController.addServo(10), webotsServo = self.WebotsController.addServo("BackLeftHip2")),
			knee = QuadrupedServo(offset = 145, physicalServo = self.ServoController.addServo(9), webotsServo = self.WebotsController.addServo("BackLeftKnee")))

		BackRightLeg = QuadrupedLeg(
			legType = QuadrupedLeg.BACK_RIGHT,
			hip1 = QuadrupedServo(offset = 135, physicalServo = self.ServoController.addServo(15), webotsServo = self.WebotsController.addServo("BackRightHip1")),
			hip2 = QuadrupedServo(offset = 128, physicalServo = self.ServoController.addServo(14), webotsServo = self.WebotsController.addServo("BackRightHip2")),
			knee = QuadrupedServo(offset = 20, physicalServo = self.ServoController.addServo(13), webotsServo = self.WebotsController.addServo("BackRightKnee")))

		self.Legs = [FrontLeftLeg, BackLeftLeg, FrontRightLeg, BackRightLeg]

	def printState(self):
		t = time()
		print(f"\nQuadruped State at {strftime(f'%I:%M:%S.{round(1000*(t%1))} %p', localtime(t))}:"
			f"\n  Gait: {self.Gaits[self.activeGait].descriptionText}"
			f"\n  Control Loop Update Rate:\t{self.updateRate:.2f} Hz"
			f"\n  Spine Motor Positions:\t{self.Spine.getMotorPositions()}")
		print("\n".join(f"  {leg.legTypeString} Leg Joint Angles:\t[{', '.join(f'{a:6.2f}°' for a in leg.getJointAngles())}]" for leg in self.Legs))
	
	def nextGait(self):
		self.activeGait = self.activeGait + 1 if self.activeGait + 1 < len(self.Gaits) else 0

	def run(self, xbox):
		print("\nPress START to continue.")

		self.WebotsController.reset()
		now = 0
		state = 0
		while True:
			timeStep = now
			now = self.WebotsController.simTime / 1000
			timeStep = now - timeStep
			self.updatePeriod = max(timeStep, 0.99 * self.updatePeriod + 0.01 * timeStep)
			self.updateRate = 0.99 * self.updateRate + 0.01 / max(1e-6, self.updatePeriod)
			
			xbox.update(now)

			if state == 1:
				if xbox.Start:
					startTime = now
					state = 2
			elif state == 2:
				self.stand()
				if not xbox.Start and now >= startTime + 1:
					startTime = now
					state = 3
			elif state == 3:
				self.runGait(xbox = xbox, time = now - startTime, gait = self.Gaits[self.activeGait])
				if xbox.Start:
					state = 0
			else:
				self.home()
				if not xbox.Start:
					state = 1

			self.Spine.runMotors((
				0.5 * xbox.DpadY + 0.5 * xbox.DpadX,
				0.5 * xbox.DpadY,
				0.5 * xbox.DpadY - 0.5 * xbox.DpadX))

			self.OrigamiController.update()
			if self.WebotsController.update():
				print("\nQuadruped Test Complete (Terminated by Webots).\n")
				break
			if xbox.Select:
				print("\nQuadruped Test Complete (Terminated by Xbox Controller).\n")
				break
		
		self.Spine.stop()
		self.OrigamiController.update()
		self.home()

	def home(self):
		for leg in self.Legs:
			leg.setJointAngles([0, 0, 0])

	def stand(self, bodyHeight = 7):
		for leg in self.Legs:
			leg.setPose2D(0, -bodyHeight)

	def runGait(self, xbox, time, gait):
		phase = time * gait.stepFrequency
		stepHeight = gait.minStepHeight + (gait.maxStepHeight - gait.minStepHeight) * xbox.RightTrigger
		bodyHeight = gait.maxBodyHeight + (gait.minBodyHeight + stepHeight - gait.maxBodyHeight) * xbox.LeftTrigger

		for i, leg in enumerate(self.Legs):
			u, v = gait.trajectory(phase, i)
			
			x = gait.xStride * xbox.LeftX * u
			y = gait.yStride * xbox.LeftY * u

			Rc = cos(pi * gait.spin * u * xbox.RightX)
			Rs = sin(pi * gait.spin * u * xbox.RightX)
			x += Rc * leg.Home[0] + Rs * leg.Home[1]
			y += Rc * leg.Home[1] - Rs * leg.Home[0]
			
			x -= gait.xSway * cos(2 * pi * phase)
			y += gait.ySway * sin(4 * pi * phase)
			x += gait.xStance if leg.isRightLeg else -gait.xStance
			y += gait.yStance if leg.isFrontLeg else -gait.yStance

			z = stepHeight * v - bodyHeight
			z += y * gait.pitch * xbox.RightY

			if xbox.A:
				leg.setPose2D(y - leg.Home[1], z)
			else:
				leg.setPose(x, y, z)
