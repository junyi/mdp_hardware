label = raw_input()

with open('ir%s.csv' % label, 'r') as f:
	with open('ir%s_avg.csv' % label, 'w') as g:
		count = 0
		total = 0
		key = 0
		for line in f:
			key, value = [int(float(i)) for i in line.strip().split(',')]
			total += value
			count += 1
			if count == 100:
				g.write("%d, %f\n" % (key, total/100))
				total = 0
				count = 0
