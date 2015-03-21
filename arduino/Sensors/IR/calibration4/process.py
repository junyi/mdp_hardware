label = raw_input()

with open('ir%s.csv' % label, 'r') as f:
	with open('ir%s_avg.csv' % label, 'w') as g:
		count = 0
		l = []
		key = 0
		for line in f:
			key, value = [int(float(i)) for i in line.strip().split(',')]
			l.append(value)
			count += 1
			if count == 11:
				g.write("%d, %f\n" % (key, sorted(l)[5]))
				total = 0
				count = 0
