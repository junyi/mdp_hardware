import serial
import sys

print 'Enter the current IR label:'
label = raw_input()
# label = 1

while True:
	print 'Enter the current range:'
	r = input()
	if r == -1:
		print 'Exiting...'
		sys.exit()

	total = 25
	with open('ir%s.csv' % label, 'a') as f:
		l = []
		ser = serial.Serial(2)
		while True:
			reading = ser.readline()
			try:
				initial = float(reading)
				print "Initial: ", initial
				x = raw_input()
				if x.lower() == 'y':
					break
			except ValueError:
				continue

		lowerbound = initial - 30
		upperbound = initial + 30
		count = 0
		while count < 100:
			try:
				reading = ser.readline()
				value = float(reading)
				if lowerbound <= value <= upperbound:
					s = "%s, %s" % (r, value)
					f.write(s + "\n")
					print s
					count += 1
			except ValueError:
				pass
		# average = []
		# for i in xrange(5):
		# 	while count < total:
		# 		reading = ser.readline()
		# 		try:
		# 			l.append(float(reading.strip()))
		# 			count += 1
		# 		except ValueError:
		# 			pass
		# 		# print reading
		# 	v = sum(l)/count

		# 	# v = sum(sorted(l)[40:60])/20
		# 	print v
		# 	l = []
		# 	count = 0
		# 	average.append(v)
		# final = sorted(average)[2]
		# print "Final", final
		# f.write("%s, %s\n" % (r, final))
		ser.close()


