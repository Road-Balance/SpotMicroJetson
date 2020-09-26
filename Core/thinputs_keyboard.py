

from time import sleep
from threading import Thread
import queue
import time
import getch

# from inputs import get_key


class ThreadedInputsKeyBoard:
	NOMATCH = 'No Match'
	
	def __init__(self, inputDict={}):
		# Initialise gamepad command dictionary.
		# Add gamepad commands using the append method before executing the start method.
		self.gamepadInputs = inputDict
		self.lastEventCode = self.NOMATCH
		# Initialise the thread status flag
		self.stopped = False
		self.q = queue.LifoQueue()

	def start(self):
		# Start the thread to poll gamepad event updates
		print("\nGetch Thread Starts\n")
		t = Thread(target=self.gamepad_update, args=())
		t.daemon = True
		t.start()
		
	def gamepad_update(self):
		while True:
			# Should the thread exit?
			if self.stopped:
				print("\nGetch Thread Ends...")
				return
			# Code execution stops at the following line until a gamepad event occurs.
			char = getch.getch()
			print("input : ", char)
			self.lastEventCode = char
			self.q.put(char)

	def read(self):
		# Return the latest command from gamepad event
		if not self.q.empty():
			newCommand = self.q.get()
			while not self.q.empty():
				trashBin = self.q.get()
			return newCommand #, self.gamepadInputs[newCommand]
		else:
			return self.NOMATCH, 0

	def stop(self):
		# Stop the game pad thread
		self.stopped = True
		
	def append_command(self, newCommand, newValue):
		# Add new controller command to the list
		if newCommand not in self.gamepadInputs:
			self.gamepadInputs[newCommand] = newValue
		else:
			print('New command already exists')
		
	def delete_command(self, commandKey):
		# Remove controller command from list
		if commandKey in self.gamepadInputs:
			del self.gamepadInputs[commandKey]
		else:
			print('No command to delete')

	def command_value(self, commandKey):
		# Get command value
		if commandKey in self.gamepadInputs:
			return self.gamepadInputs[commandKey]
		else:
			return None

if __name__ == "__main__":
	keyboard = ThreadedInputsKeyBoard()
	keyboard.start()
	while 1:
		commandValue = keyboard.read()
		if commandValue == "q":
			keyboard.stop()
			break