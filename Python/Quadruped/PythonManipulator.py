
from math import sin, cos, sqrt

class PythonManipulator:
	@staticmethod
	def ikin(cManip, cN, cP, cQ):
		Robot = list(cManip)
		n = int(cN.value)
		p = list(cP)
		q = list(cQ)
		S = Robot[: 6*n]
		M = Robot[24*n: 24*n+12]

		ikina(S, M, n, p, 1e-4, 20e-3, 100, q)

		for i, qi in enumerate(q):
			cQ[i] = qi

# WARNING: These functions are intended to mirror the C++ implementation as closely as
# possible, and do not necessarily follow typical Python conventions or best practices.

def ikina(S, M, n, p, tol, k, maxSteps, q):
	tol *= tol

	J = [0] * 6 * n
	T = [0] * 12
	L = [0] * 9
	e = [0] * 3

	for step in range(maxSteps):
		# Calculate Jacobian and Pose
		jacobPose(S, q, n, J, T)

		T[3] += T[0] * M[3] + T[1] * M[7] + T[2] * M[11]
		T[7] += T[4] * M[3] + T[5] * M[7] + T[6] * M[11]
		T[11] += T[8] * M[3] + T[9] * M[7] + T[10] * M[11]

		# Calculate Position Error
		e[0] = p[0] - T[3]
		e[1] = p[1] - T[7]
		e[2] = p[2] - T[11]

		# Check Position Error Tolerance
		if e[0] * e[0] + e[1] * e[1] + e[2] * e[2] < tol:
			return

		# Calculate Analytical Jacobian
		J[3] = S[3] + T[11] * S[1] - T[7] * S[2]
		J[4] = S[4] + T[3] * S[2] - T[11] * S[0]
		J[5] = S[5] + T[7] * S[0] - T[3] * S[1]

		# L = Ja * Ja' + lambda * I
		L[0] = J[3] * J[3] + k
		L[1] = J[3] * J[4]
		L[2] = J[3] * J[5]
		L[4] = J[4] * J[4] + k
		L[5] = J[4] * J[5]
		L[8] = J[5] * J[5] + k

		for i in range(1, n):
			J[6*i+3] += T[11] * J[6*i+1] - T[7] * J[6*i+2]
			J[6*i+4] += T[3] * J[6*i+2] - T[11] * J[6*i]
			J[6*i+5] += T[7] * J[6*i] - T[3] * J[6*i+1]

			L[0] += J[6*i+3] * J[6*i+3]
			L[1] += J[6*i+3] * J[6*i+4]
			L[2] += J[6*i+3] * J[6*i+5]
			L[4] += J[6*i+4] * J[6*i+4]
			L[5] += J[6*i+4] * J[6*i+5]
			L[8] += J[6*i+5] * J[6*i+5]

		# q += J' * ((J * J' + lambda * I) \ e)
		chol(L, 3)
		cholSolve(L, e, 3, e)

		for i in range(n):
			q[i] += J[6*i+3] * e[0] + J[6*i+4] * e[1] + J[6*i+5] * e[2]

def jacobPose(S, q, n, J, T):
	temp = [0.0]*12
	tw2ht(S[:6], q[0], T)
	for i in range(1, n):
		adjoint(T, S[6*i:6*i+6], temp)
		J[6*i:6*i+6] = temp[:6]
		tw2ht(S[6*i:6*i+6], q[i], temp)
		htMul(T, temp, T)

def htMul(Ta, Tb, Tp):
	c0 = Ta[0] * Tb[0] + Ta[1] * Tb[4] + Ta[2] * Tb[8]
	c1 = Ta[0] * Tb[1] + Ta[1] * Tb[5] + Ta[2] * Tb[9]
	c2 = Ta[0] * Tb[2] + Ta[1] * Tb[6] + Ta[2] * Tb[10]
	c3 = Ta[0] * Tb[3] + Ta[1] * Tb[7] + Ta[2] * Tb[11] + Ta[3]
	Tp[0] = c0
	Tp[1] = c1
	Tp[2] = c2
	Tp[3] = c3

	c0 = Ta[4] * Tb[0] + Ta[5] * Tb[4] + Ta[6] * Tb[8]
	c1 = Ta[4] * Tb[1] + Ta[5] * Tb[5] + Ta[6] * Tb[9]
	c2 = Ta[4] * Tb[2] + Ta[5] * Tb[6] + Ta[6] * Tb[10]
	c3 = Ta[4] * Tb[3] + Ta[5] * Tb[7] + Ta[6] * Tb[11] + Ta[7]
	Tp[4] = c0
	Tp[5] = c1
	Tp[6] = c2
	Tp[7] = c3

	c0 = Ta[8] * Tb[0] + Ta[9] * Tb[4] + Ta[10] * Tb[8]
	c1 = Ta[8] * Tb[1] + Ta[9] * Tb[5] + Ta[10] * Tb[9]
	c2 = Ta[8] * Tb[2] + Ta[9] * Tb[6] + Ta[10] * Tb[10]
	c3 = Ta[8] * Tb[3] + Ta[9] * Tb[7] + Ta[10] * Tb[11] + Ta[11]
	Tp[8] = c0
	Tp[9] = c1
	Tp[10] = c2
	Tp[11] = c3

def adjoint(T, Va, Vb):
	v0 = Va[0]
	v1 = Va[1]
	v2 = Va[2]

	v3 = (T[0] * Va[3] + T[1] * Va[4] + T[2] * Va[5]
		+ (T[7] * T[8] - T[11] * T[4]) * v0
		+ (T[7] * T[9] - T[11] * T[5]) * v1
		+ (T[7] * T[10] - T[11] * T[6]) * v2)

	v4 = (T[4] * Va[3] + T[5] * Va[4] + T[6] * Va[5]
		+ (T[11] * T[0] - T[3] * T[8]) * v0
		+ (T[11] * T[1] - T[3] * T[9]) * v1
		+ (T[11] * T[2] - T[3] * T[10]) * v2)

	v5 = (T[8] * Va[3] + T[9] * Va[4] + T[10] * Va[5]
		+ (T[3] * T[4] - T[7] * T[0]) * v0
		+ (T[3] * T[5] - T[7] * T[1]) * v1
		+ (T[3] * T[6] - T[7] * T[2]) * v2)

	Vb[0] = T[0] * v0 + T[1] * v1 + T[2] * v2
	Vb[1] = T[4] * v0 + T[5] * v1 + T[6] * v2
	Vb[2] = T[8] * v0 + T[9] * v1 + T[10] * v2
	Vb[3] = v3
	Vb[4] = v4
	Vb[5] = v5

def tw2ht(V, theta, T):
	s = sin(theta)
	c = 1.0 - cos(theta)

	a = theta - s

	s0 = s * V[0]
	s1 = s * V[1]
	s2 = s * V[2]
	c0 = c * V[0]
	c1 = c * V[1]
	c2 = c * V[2]

	v0 = V[0] * V[0]
	v1 = V[1] * V[1]
	v2 = V[2] * V[2]

	w00 = v1 + v2
	w11 = v0 + v2
	w22 = v0 + v1
	w01 = V[0] * V[1]
	w02 = V[0] * V[2]
	w12 = V[1] * V[2]

	# R = eye(3) + s(theta) * w + (1 - c(theta)) * w * w
	v0 = c * w01
	v1 = c * w02
	v2 = c * w12

	T[0] = 1.0 - c * w00
	T[5] = 1.0 - c * w11
	T[10] = 1.0 - c * w22

	T[4] = v0 + s2
	T[8] = v1 - s1
	T[9] = v2 + s0

	T[1] = v0 - s2
	T[2] = v1 + s1
	T[6] = v2 - s0

	# p = (theta * eye(3) + (1 - c(theta)) * w + (theta - s(theta)) * w * w) * V[3:5]
	w00 *= a
	w11 *= a
	w22 *= a
	w01 *= a
	w02 *= a
	w12 *= a

	T[3] = (theta - w00) * V[3] + (w01 - c2) * V[4] + (w02 + c1) * V[5]
	T[7] = (w01 + c2) * V[3] + (theta - w11) * V[4] + (w12 - c0) * V[5]
	T[11] = (w02 - c1) * V[3] + (w12 + c0) * V[4] + (theta - w22) * V[5]

def chol(L, n):
	for i in range(n):
		sum = L[n*i+i]
		for k in range(i - 1, -1, -1):
			sum -= L[n*i+k] * L[n*i+k]
		L[n*i+i] = sqrt(sum)
		for j in range(i + 1, n):
			sum = L[n*i+j]
			for k in range(i - 1, -1, -1):
				sum -= L[n*i+k] * L[n*j+k]
			L[n*j+i] = sum / L[n*i+i]

def cholSolve(L, b, n, x):
	for i in range(n):
		sum = b[i]
		for k in range(i - 1, -1, -1):
			sum -= L[n * i + k] * x[k]
		x[i] = sum / L[n * i + i]
	for i in range(n - 1, -1, -1):
		sum = x[i]
		for k in range(i + 1, n):
			sum -= L[n * k + i] * x[k]
		x[i] = sum / L[n * i + i]
