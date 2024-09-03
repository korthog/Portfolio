
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation

def animate(pos, timestep, fileName = None):
	fig = plt.figure()
	ax = plt.axes(projection='3d', xlim=(-1, 1), ylim=(-1, 1), zlim=(-1, 1))
	links, = ax.plot3D([], [], [], lw=2)
	joints, = ax.plot3D([], [], [], linestyle='', marker='o')

	def init():
		links.set_data_3d([], [], [])
		joints.set_data_3d([], [], [])
		return links, joints

	def updateAnimation(i):
		x = pos[i, 0::3]
		y = pos[i, 1::3]
		z = pos[i, 2::3]
		links.set_data_3d(x, y, z)
		joints.set_data_3d(x, y, z)
		return links, joints

	numFrames, numJoints = pos.shape
	a = FuncAnimation(fig, updateAnimation, init_func=init, frames=numFrames, interval=timestep, blit=True)
	plt.xlabel('X')
	plt.ylabel('Y')
	plt.ylabel('Z')

	if fileName is not None:
		if not fileName.lower().endswith(('.gif')):
			fileName += '.gif'
		a.save(fileName)

	plt.show()
