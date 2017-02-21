# finite state machine class

class FSM:

	current_state = None
	states = {'Go Forward', 'Hit Left', 'Hit Right', 'Hit Center', 'Wheel Drop'}

	def __init__(self):
		self.current_state = 'Go Forward'
		self.counter = 0

	def getCurrentState(self):
		return self.current_state

	def setCurrentState(self, state):
		if state in states:
			self.current_state = state