
from math import radians, degrees, sqrt, acos, atan2, pi
from Manipulator import Manipulator

class QuadrupedLeg:
	FRONT_LEFT = 0
	FRONT_RIGHT = 1
	BACK_RIGHT = 2
	BACK_LEFT = 3

	def __init__(self, legType, hip1, hip2, knee):
		self.legType = legType
		self.isFrontLeg = (True, True, False, False)[legType]
		self.isRightLeg = (False, True, True, False)[legType]
		self.legTypeString = ("Front Left", "Front Right", "Back Right", "Back Left")[legType]
		self.joints = (hip1, hip2, knee)
		sx = 1 if self.isRightLeg else -1
		sy = 1 if self.isFrontLeg else -1
		self.manip = Manipulator()
		self.manip.addRevolute(w = [0, sy, 0], p = [0.0446 * sx, 0.0951 * sy, 0])
		self.manip.addRevolute(w = [sx, 0, 0], p = [0.0869 * sx, 0.1243 * sy, 0])
		self.manip.addRevolute(w = [sx, 0, 0], p = [0.1027 * sx, 0.1243 * sy, -0.0889])
		home = (0.1027 * sx, 0.1243 * sy, -0.1842)
		self.manip.setHome(home)
		self.Home = [meter2inch(x) for x in home]

	def setPose(self, x, y, z):
		p = [inch2meter(a) for a in (x, y, z)]
		q = [radians(joint.angle) for joint in self.joints]

		if self.legType == QuadrupedLeg.FRONT_RIGHT or self.legType == QuadrupedLeg.BACK_LEFT:
			if q[2] > 0:
				q[1:] = self.pose2D(y, z)
		elif q[2] < 0:
			q[1:] = self.pose2D(y, z)

		q = self.manip.ikin(p, q)
		self.setJointAngles([degrees(qi) for qi in q])

	def pose2D(self, y, z, L1 = 3.5, L2 = 3.75):
		if not self.isFrontLeg:
			y = -y
		hip2Angle = atan2(-z, y) - acos(max(-1, min(1, (y*y + z*z + L1*L1 - L2*L2) / (2*L1*sqrt(y*y + z*z))))) - 0.5 * pi
		kneeAngle = acos(max(-1, min(1, (y*y + z*z - L1*L1 - L2*L2) / (2*L1*L2))))
		if self.legType == QuadrupedLeg.FRONT_RIGHT or self.legType == QuadrupedLeg.BACK_LEFT:
			return -hip2Angle, -kneeAngle
		else:
			return hip2Angle, kneeAngle

	def setPose2D(self, y, z, L1 = 3.5, L2 = 3.75):
		hip2, knee = self.pose2D(y, z, L1, L2)
		self.setJointAngles([0, degrees(hip2), degrees(knee)])

	def getJointAngles(self):
		return [joint.angle for joint in self.joints]

	def setJointAngles(self, jointAngles):
		try:
			for i, joint in enumerate(self.joints):
				joint.setAngle(jointAngles[i])
		except:
			jointString = ("Hip 1", "Hip 2", "Knee")[i]
			print(f"\nJoint Angle Error in {self.legTypeString} leg. {jointString} joint set to {jointAngles[i]:.2f}°.\n")
			raise

def inch2meter(x):
	return 0.0254 * x

def meter2inch(x):
	return x / 0.0254
