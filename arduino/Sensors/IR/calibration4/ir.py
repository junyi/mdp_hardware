import serial
import sys
from collections import Counter

MOD_FILTER_SIZE = 50

def mode(samples):
    fmode               = 0
    i                  = 0
    count              = 0
    maxCount   = 0
    bimodal    = 0

    while count > maxCount:
        fmode           = samples[i]
        maxCount        = count
        bimodal         = 0

    if count == 0:
    	i+=1

    if count == maxCount:
        bimodal = 1

    if fmode == 0 or bimodal == 1:
        fmode = samples[(MOD_FILTER_SIZE / 2)]

    return fmode


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
		ser = serial.Serial(2)
		times = 0
		while times < 11:
			l = []
			count = 0
			while count < MOD_FILTER_SIZE:
				try:
					reading = ser.readline()
					value = float(reading)
					l.append(value)
					count += 1
				except ValueError:
					pass
			print l
			c = Counter(l).most_common(1)
			print c
			f.write("%s, %s\n" % (r, c[0][0]))
			times += 1
		
		ser.close()


