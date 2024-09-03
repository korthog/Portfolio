
import numpy as np
from sys import platform
from os.path import dirname, realpath
from ctypes import *

class Manipulator:
	try:
		_manipulatorDll = CDLL(f"{dirname(dirname(realpath(__file__)))}/lib/Manipulator.{'dll' if platform == 'win32' else 'so'}")
		_manipulatorDll.ikin.argtypes = [POINTER(c_double), c_int, POINTER(c_double), POINTER(c_double)]
		_manipulatorDll.ikin.restype = None
		_manipulatorDll.simStep.argtypes = [POINTER(c_double), c_int, c_double, POINTER(c_double), POINTER(c_double)]
		_manipulatorDll.simStep.restype = None
		_manipulatorDll.fkin.argtypes = [POINTER(c_double), c_int, POINTER(c_double), POINTER(c_double)]
		_manipulatorDll.fkin.restype = None
	except:
		print("\nWARNING: Unable to load Manipulator DLL. Continuing with Python implementation.")
		from PythonManipulator import PythonManipulator as _manipulatorDll

	default_S = [0, 0, 1, 0, 0, 0]
	default_M = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0]
	default_G = [0] * 6

	def __init__(self):
		self.N = 0
		self.S = []
		self.M = []
		self.G = []
		self.Home = Manipulator.default_M
		manip = self.S + self.G + self.M + self.Home
		self._cManip = (c_double*len(manip))(*manip)
		self._cN = c_int(self.N)

	def addRevoluteDH(self, d = 0, theta = 0, r = 0, alpha = 0, M = default_M, G = default_G):
		w = [self.Home[2], self.Home[6], self.Home[10]]
		p = [self.Home[3], self.Home[7], self.Home[11]]
		S = w + list(np.cross(p, w))
		self.Home = np.matmul(self.Home, dh2ht(d, theta, r, alpha))
		self.addRevoluteJoint(S, M, G)

	def addPrismatic(self, v = [0, 0, 1], M = default_M, G = default_G):
		if len(v) != 3:
			raise ValueError("Manipulator Argument Length Error: v must contain 3 elements.")
		elif abs(np.dot(v, v) - 1) > 1e-6:
			raise ValueError("Manipulator Argument Length Error: v must be a unit vector.")
		self.addJoint([0, 0, 0] + v, M, G)

	def addRevolute(self, w = [0, 0, 1], p = [0, 0, 0], M = default_M, G = default_G):
		if len(w) != 3:
			raise ValueError("Manipulator Argument Length Error: w must contain 3 elements.")
		elif abs(np.dot(w, w) - 1) > 1e-6:
			raise ValueError("Manipulator Argument Length Error: w must be a unit vector.")
		elif len(p) != 3:
			raise ValueError("Manipulator Argument Length Error: p must contain 3 elements.")
		else:
			self.Home[3] = p[0]
			self.Home[7] = p[1]
			self.Home[11] = p[2]
			self.addJoint(w + list(np.cross(p, w)), M, G)

	def addJoint(self, S = default_S, M = default_M, G = default_G):
		if len(S) != 6:
			raise ValueError("Manipulator Argument Length Error: S must contain 6 elements.")
		elif len(M) != 12:
			raise ValueError("Manipulator Argument Length Error: M must contain 12 elements.")
		elif len(G) != 6:
			raise ValueError("Manipulator Argument Length Error: G must contain 6 elements.")
		else:
			self.N += 1
			self.S += S
			self.M += M
			self.G += G
			manip = self.S + self.G + self.M + self.Home
			self._cManip = (c_double*len(manip))(*manip)
			self._cN = c_int(self.N)

	def setHome(self, M):
		if len(M) == 3:
			self.Home = [1, 0, 0, M[0], 0, 1, 0, M[1], 0, 0, 1, M[2]]
		elif len(M) == 12:
			self.Home = M
		else:
			raise ValueError("Manipulator Argument Length Error: M must contain either 3 or 12 elements.")
		manip = self.S + self.G + self.M + self.Home
		self._cManip = (c_double*len(manip))(*manip)

	def ikin(self, p, q = []):
		if len(p) != 3:
			raise ValueError("Manipulator Argument Length Error: p must contain 3 elements.")
		else:
			if len(q) != self.N:
				q = [0] * self.N
			cP = (c_double*len(p))(*p)
			cQ = (c_double*len(q))(*q)
			Manipulator._manipulatorDll.ikin(self._cManip, self._cN, cP, cQ)
			return list(cQ)

	def simStep(self, q, dq, u, timeStep):
		if len(q) != self.N:
			raise ValueError(f"Manipulator Argument Length Error: q must contain {self.N} elements.")
		elif len(dq) != self.N:
			raise ValueError(f"Manipulator Argument Length Error: dq must contain {self.N} elements.")
		elif len(u) != self.N:
			raise ValueError(f"Manipulator Argument Length Error: u must contain {self.N} elements.")
		else:
			q += dq
			cQ = (c_double*len(q))(*q)
			cU = (c_double*len(u))(*u)
			cDt = c_double(timeStep)
			Manipulator._manipulatorDll.simStep(self._cManip, self._cN, cDt, cU, cQ)
			qdq = list(cQ)
			return qdq[:self.N], qdq[self.N:]

	def fkin(self, p0, q):
		cP = (c_double*len(p0))(*p0)
		cQ = (c_double*len(q))(*q)
		Manipulator._manipulatorDll.fkin(self._cManip, self._cN, cQ, cP)
		return list(cP)
	
def dt2ht(d = 0, theta = 0, r = 0, alpha = 0):
	theta = np.radians(theta)
	sinTheta = np.sin(theta)
	cosTheta = np.cos(theta)
	alpha = np.radians(alpha)
	sinAlpha = np.sin(alpha)
	cosAlpha = np.cos(alpha)
	return np.mat([
		[cosTheta, -sinTheta*cosAlpha, sinTheta*sinAlpha, r*cosTheta],
		[sinTheta, cosTheta*cosAlpha, -cosTheta*sinAlpha, r*sinTheta],
		[0, sinAlpha, cosAlpha, d],
		[0, 0, 0, 1]])
