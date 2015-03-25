label = raw_input()

with open('ir%s.csv' % label, 'r') as f:
	with open('ir%s_avg.csv' % label, 'w') as g:
		count = 0
		l = []
		l2 = []
		key = 0
		for line in f:
			key, value1, value2 = [int(float(i)) for i in line.strip().split(',')]
			l.append(value1)
			l2.append(value2)
			count += 1
			if count == 11:
				g.write("%d, %f, %f\n" % (key, sorted(l)[5], sorted(l2)[5]))
				total = 0
				count = 0
