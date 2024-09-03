
import os

if os.name == "nt":
	import ctypes

	class _XinputGamepad(ctypes.Structure):
		_fields_ = [('Buttons', ctypes.c_ushort),
			('LeftTrigger', ctypes.c_ubyte),
			('RightTrigger', ctypes.c_ubyte),
			('LeftX', ctypes.c_short),
			('LeftY', ctypes.c_short),
			('RightX', ctypes.c_short),
			('RightY', ctypes.c_short)]

	class _XinputState(ctypes.Structure):
			_fields_ = [('Packet', ctypes.c_ulong), ('Gamepad', _XinputGamepad)]
else:
	import struct

class XboxController:
	ButtonNames = ('A', 'B', 'X', 'Y', 'LB', 'RB', 'Start', 'Select')

	def __init__(self, userIndex = 0):
		self.userIndex = userIndex
		self._clearState()
		self._onButton = [lambda: None] * 2 * len(XboxController.ButtonNames)

	def onButtonReleased(self, button, function):
		self._onButton[2 * XboxController.ButtonNames.index(button)] = function

	def onButtonPressed(self, button, function):
		self._onButton[2 * XboxController.ButtonNames.index(button) + 1] = function

	def printState(self):
		print("\nXbox Controller Disconnected." if not self.Connected else (
			"\nXbox Controller State:"
			f"\n  LeftX:      {self.LeftX:6.3f}\tLeftY:       {self.LeftY:6.3f}"
			f"\n  RightX:     {self.RightX:6.3f}\tRightY:      {self.RightY:6.3f}"
			f"\n  LeftTrigger:{self.LeftTrigger:6.3f}\tRightTrigger:{self.RightTrigger:6.3f}"
			f"\n  DpadX:      {self.DpadX:2}    \tDpadY:       {self.DpadY:2}"
			f"\n  A:          {self.A:2}    \tB:           {self.B:2}"
			f"\n  X:          {self.X:2}    \tY:           {self.Y:2}"
			f"\n  LB:         {self.LB:2}    \tRB:          {self.RB:2}"
			f"\n  Start:      {self.Start:2}    \tSelect:      {self.Select:2}"))

	def _clearState(self):
		self.Connected = False

		for a in ('LeftX', 'LeftY', 'RightX', 'RightY', 'LeftTrigger', 'RightTrigger', 'DpadX', 'DpadY'):
			self.__setattr__(a, 0)

		for b in XboxController.ButtonNames:
			self.__setattr__(b, int(0))
		
		if os.name == "nt":
			self._previousButtons = int(0)

	if os.name == "nt":
		_xInputDll = getattr(ctypes.windll, "XInput1_4.dll")

		def __enter__(self):
			return self

		def __exit__(self, *args):
			return

		def update(self, *args):
			state = _XinputState()
			if XboxController._xInputDll.XInputGetState(ctypes.c_uint32(self.userIndex), ctypes.byref(state)):
				self._clearState()
			else:
				self.Connected = True
				for s in ('LeftX', 'LeftY', 'RightX', 'RightY'):
					self.__setattr__(s, XboxController._deadband(state.Gamepad.__getattribute__(s)))
				self.LeftTrigger = state.Gamepad.LeftTrigger / 0xFF
				self.RightTrigger = state.Gamepad.RightTrigger / 0xFF

				buttons = int(state.Gamepad.Buttons)
				self.DpadX = (0, -1, 1, 0)[buttons >> 2 & 0x03]
				self.DpadY = (0, 1, -1, 0)[buttons & 0x03]

				for i, mask in enumerate((0x1000, 0x2000, 0x4000, 0x8000, 0x100, 0x200, 0x20, 0x10)):
					self.__setattr__(XboxController.ButtonNames[i], int(not not (buttons & mask)))
					if ~self._previousButtons & buttons & mask:
						self._onButton[2 * i + 1]()
					elif self._previousButtons & ~buttons & mask:
						self._onButton[2 * i]()
				self._previousButtons = buttons

	else:
		def __enter__(self):
			self._connect()
			return self

		def __exit__(self, *args):
			os.close(self._fid)

		def _connect(self):
			try:
				self._fid = os.open(f"/dev/input/js{self.userIndex}", os.O_RDONLY | os.O_NONBLOCK)
			except (FileNotFoundError, PermissionError):
				return
			else:
				self.Connected = True

		def update(self, *args):
			while self.Connected:
				try:
					eventValue, eventType, eventIndex = struct.unpack_from("hBB", os.read(self._fid, 8), offset = 4)
				except BlockingIOError:
					return
				except (FileNotFoundError, OSError):
					self._clearState()
					return

				if eventType == 1:
					self.__setattr__(XboxController.ButtonNames[eventIndex], eventValue & 1)
					self._onButton[2 * eventIndex + (eventValue & 1)]()
				elif eventType == 2:
					match eventIndex:
						case 0: self.LeftX = XboxController._deadband(eventValue)
						case 1: self.LeftY = -XboxController._deadband(eventValue)
						case 2: self.LeftTrigger = (eventValue + 0x7FFF) / 0xFFFE
						case 3: self.RightX = XboxController._deadband(eventValue)
						case 4: self.RightY = -XboxController._deadband(eventValue)
						case 5: self.RightTrigger = (eventValue + 0x7FFF) / 0xFFFE
						case 6: self.DpadX = (eventValue > 0) - (eventValue < 0)
						case 7: self.DpadY = (eventValue < 0) - (eventValue > 0)
			else:
				self._connect()
	
	@staticmethod
	def _deadband(x, db = 0x1000, sat = 0x8000):
		if x < 0:
			return -XboxController._deadband(-x, db, sat)
		else:
			return 0 if x <= db else 1 if x >= sat else (x - db) / (sat - db)
