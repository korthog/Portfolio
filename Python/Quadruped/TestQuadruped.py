
from Quadruped import Quadruped
from XboxController import XboxController

def main():
	print("\nStarting Quadruped...")
	robot = Quadruped()
	with XboxController() as xbox:
		xbox.onButtonPressed('X', xbox.printState)
		xbox.onButtonPressed('Y', robot.printState)
		xbox.onButtonPressed('B', robot.nextGait)
		robot.run(xbox = xbox)

if __name__ == "__main__":
	main()
