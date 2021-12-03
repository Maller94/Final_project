import numpy as np

Q = np.array([[ -8.02,-8.02,6.98,-8.02],
            [ -7.03000002,-7.03,-7.03,7.97,],
            [100.,-10.,-10.,-10.],
            [ -9.7032772,-8.09920019,-8.02,-8.1883,]])

def writeToQ(q):
    with open('QTable.txt','w') as w:
        w.write(str(q))

writeToQ(Q)

def readQ():
    with open('Qtable.txt','r') as r:
        QtableRead = np.array(r.read())
    return QtableRead

print(readQ())