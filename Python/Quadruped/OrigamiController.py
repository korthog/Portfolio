
try:
	import hid
except:
	hid = None

class OrigamiController:
	def __init__(self, vid = 0xFFEF, pid = 0x0004):
		self._motorBytes = [0] * 64
		try:
			try:
				self._usbDevice = hid.Device(vid, pid)
			except:
				self._usbDevice = hid.device()
				self._usbDevice.open(vid, pid)
			self._usbDevice.write(bytes([0] + self._motorBytes))
			self._usbDevice.read(64)
			self._position = list(self._usbDevice.read(64))[::2]
		except:
			print("\nWARNING: Unable to open Origami Module USB Interface. Continuing without Origami Module motor control.")
			self._usbDevice = None
			self._position = [0] * 32
	
	def update(self):
		if self._usbDevice is None:
			return
		self._usbDevice.write(bytes([0] + self._motorBytes))
		inData = list(self._usbDevice.read(64))
		for i, pos in enumerate(self._position):
			d = (inData[i * 2] & 0xFF) - (pos & 0xFF)
			if d > 128:
				d -= 256
			elif d < -128:
				d += 256
			pos += max(-16, min(16, d))
	
	def setMotorPwm(self, channel, value):
		index = int(channel)
		if index < 0 or index > 31:
			raise ValueError(f"Origami Motor Channel Error: Attempted to set channel {index}. Channel must be in [0:31].")
		value = max(-2047, min(2047, round(2048 * value)))
		self._motorBytes[2 * index] = (value >> 8) & 0x0F
		self._motorBytes[2 * index + 1] = value & 0xFF

	def getMotorPosition(self, channel):
		return self._position[channel]
