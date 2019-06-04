#!/usr/bin/env python
import numpy as np

my_list = [x for x in range(10)]
test = np.zeros((1,10))
print(test)

for elem in my_list:
	# print(elem)
	test[0,elem] = elem

print(test)