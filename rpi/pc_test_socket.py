import socket
import string
import time
import threading
import signal
import sys
import shutil
import os
import traceback

__author__ = "Rohit"

# Dummy client code
enableCsv = False

class Test(threading.Thread):
	def __init__(self):
		threading.Thread.__init__(self)
		self.ip = "192.168.5.5" # Connecting to IP address of MDPGrp2
		self.port = 5182
		self.is_running = True
		self.num = 0
		# message = "Hello World!"
		# message = list(string.ascii_lowercase)


		# Create a TCP/IP socket
		self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.client_socket.connect((self.ip, self.port))
		# f = self.client_socket.makefile()

# Send data
	def write(self, count = 0):
		print "Enter text to send: "
		msg = raw_input()
		while True:
			# if (count == 1):
				# break
			try:
				d  = None
				if msg[0] == 'r':
					d = chr(0b00000110) + chr(0b00000000)
				elif msg[0] == 'l':
					d = chr(0b00000101) + chr(0b00000000)
				elif msg[0] == 's':
					d = chr(0b00000111) + chr(0b00000001)
				elif msg[0] == 'b':
					d = chr(0b00000100) + chr(0b00000001)		
				elif msg[0] == 'c':
					d = chr(0b00001111) + chr(0b00000000)
				elif msg[0] == 'k':
					d = chr(0b00001110) + chr(0b00000000)
				elif msg[0] == 'd':
					d = chr(0b01000111) + chr(64 + int(msg[1:]))	

				if enableCsv:
					if d is None or d is not None and (ord('1') <= ord(d[1]) <= ord('9') or d[1] in ['c', 'k']):
						pass
					else:
						try:
							shutil.copy2(os.path.join(os.getcwd(), 'test.csv'), os.path.join(os.getcwd(), 'test%03d.csv' % self.num))
							f = open('test.csv', 'w')
							f.close()
							self.num += 1
						except Exception:
							print traceback.print_exc()

				if d is not None:
					self.client_socket.send('h' + d)
				else:
					self.client_socket.send('h' + msg)

				print "sending: ", msg
				# print "Enter text to send: "
				msg = raw_input("Enter a value:")
				# count += 1
			except:
				pass
		print "quit write()"

	def linesplit(self, socket):
		    buffer = socket.recv(2048)
		    buffering = True
		    while buffering:
		        if "\n" in buffer:
		            (line, buffer) = buffer.split("\n", 1)
		            yield line + "\n"
		        else:
		            more = socket.recv(2048)
		            if not more:
		                buffering = False
		            else:
		                buffer += more
		    if buffer:
		        yield buffer

	# Receive data
	def receive(self):
		while True:
			for data in self.linesplit(self.client_socket):
			# data = self.client_socket.recv(2048)
				if not data:
					print "quitting..."
					break
			# print "Data received: %s " % data
				if enableCsv and data.startswith('##'):
					try:
						l = data[2:].split()
						print l
						with open('test.csv', 'a') as f:
							f.write(', '.join([str(i) for i in l]) + '\n')
					except ValueError:
						pass

				print data
			# while True:
			# 	if (data == 'q' or len(data) == 0):
			# 		break
		print "quit receive()"
		self.client_socket.close()
		
	def keep_main(self):
		while self.is_running:
			time.sleep(0.5)




if __name__ == "__main__":
	test = Test()

	if len(sys.argv) > 1:
		if sys.argv[1] == '-c':
			enableCsv = True

	rt = threading.Thread(target = test.receive)
	wt = threading.Thread(target = test.write)

	rt.daemon = True
	wt.daemon = True

	rt.start()
	wt.start()
	print "start rt and wt"

	test.keep_main()

	def signal_handler(signal, frame):
		test.client_socket.close()
		test.is_running = False

	signal.signal(signal.SIGINT, signal_handler)

	# rt.join()
	# wt.join()
	# print "stop rt and wt"

	# Close connections
	self.client_socket.close()
	print "End of client program"