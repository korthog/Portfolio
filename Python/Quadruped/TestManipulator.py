
from time import time
from numpy import ndarray, rad2deg
from Manipulator import Manipulator
from PythonManipulator import PythonManipulator, chol, cholSolve
from ManipulatorAnimation import animate

def main():
	manip = Manipulator()
	manip.addRevoluteJoint([0, 0, 1, 0, 0, 0], [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0.2], [0.0405, 0.0405, 0.0009, 3.0, 3.0, 3.0])
	manip.addRevoluteJoint([0, -1, 0, 0.4, 0, 0], [0, 0, 1, 0.1, 0, 1, 0, 0, -1, 0, 0, 0.4], [0.0017, 0.0017, 0.0002, 0.5, 0.5, 0.5])
	manip.addRevoluteJoint([1, 0, 0, 0, 0.4, 0], [0, 0, 1, 0.3, 0, 1, 0, 0, -1, 0, 0, 0.4], [0.0017, 0.0017, 0.0002, 0.5, 0.5, 0.5])
	manip.addRevoluteJoint([0, -1, 0, 0.4, 0, -0.4], [1, 0, 0, 0.4, 0, 1, 0, 0, 0, 0, 1, 0.485], [0.0013, 0.0013, 0.0002, 0.5, 0.5, 0.5])
	manip.addRevoluteJoint([0, 0, 1, 0, -0.4, 0], [1, 0, 0, 0.4, 0, 1, 0, 0, 0, 0, 1, 0.655], [0.0013, 0.0013, 0.0002, 0.5, 0.5, 0.5])
	manip.addRevoluteJoint([0, -1, 0, 0.74, 0, -0.4], [0, 0, 1, 0.463, 0, 1, 0, 0, -1, 0, 0, 0.74], [0.0006, 0.0006, 0.0001, 0.4, 0.4, 0.4])
	manip.setHomeConfiguration([1, 0, 0, 0.526, 0, 0, -1, 0, 0, 1, 0, 0.74])

	q = [0.4880, -0.4626, -0.2681, -0.5295, 0.0664, 0.1018]
	dq = [0.3147, 0.4058, -0.3730, 0.4134, 0.1324, -0.4025]

	#u = [0.022016855389392, 9.196881485245441, 0.590600311227626, 2.630885616901889, -0.043062937302186, 0.138587361215873]
	u = [0] * manip.N
	p0 = [0, 0, 0.4, 0, 0, 0.4, 0.4, 0, 0.4, 0.4, 0, 0.4, 0.4, 0, 0.74, 0.526, 0, 0.74]
	p = ndarray((250, 3 * manip.N))

	print("Initial State: " + str(q))

	tic = time()
	for i in range(250):
		p[i, :] = manip.fkin(p0, q)
		q, dq = manip.simStep(q, dq, u, 20e-3)
	toc = time() - tic

	print("Final State: " + str(q))
	print("\nSimulation completed in " + str(toc) + " seconds.\n")

	animate(p, 20, 'ManipulatorAnimation')

def testIkin():
	manip = Manipulator()
	manip.addRevoluteJoint()
	manip.addRevoluteJoint(w=[-1, 0, 0], p=[-0.0608, 0, 0])
	manip.addRevoluteJoint(w=[1, 0, 0], p=[-0.0608, 0, 0.0889])
	manip.setHomeConfiguration(M=[1, 0, 0, -0.0608, 0, 1, 0, 0, 0, 0, 1, 0.1842])

	target = [0, -5, 0]
	p = [0.0254 * target[0] - 0.0608, 0.0254 * target[1], 0.0254 * target[2]]
	q1 = manip.ikin(p=p)

	q2 = [0]*3
	PythonManipulator.ikin(cManip=manip._cManip, cN=manip._cN, cP=p, cQ=q2)

	print(f"Inverse Kinematics Test: {'PASS' if ArraysEqual(q1, q2, 1e-9) else 'FAIL'}")

def testChol():
	A = [	3.003217436238645, 1.659068572274807, 2.432642921790789, 1.279492052255865,
			1.659068572274807, 1.289834014392391, 1.450692528403469, 0.657039422017651,
			2.432642921790789, 1.450692528403469, 2.847823143596547, 1.391123829732284,
			1.279492052255865, 0.657039422017651, 1.391123829732284, 0.969167182059849]

	b = [0.351659507062997, 0.830828627896291, 0.585264091152724, 0.549723608291140]
	d = [-1.280018108747851, 2.100872981518109, -0.595963245646154, 1.688252489992340]
	n = len(d)
	x = [0]*n

	chol(A, n)
	cholSolve(A, b, n, x)
	print(f"Cholesky Test: {'PASS' if ArraysEqual(d, x, 1e-9) else 'FAIL'}")

def ArraysEqual(a, b, tol):
	if len(a) != len(b):
		return False
	for i, ai in enumerate(a):
		if abs(ai - b[i]) > tol:
			return False
	return True

if __name__ == "__main__":
	testChol()
	testIkin()
	#main()
