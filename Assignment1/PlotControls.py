import numpy as np
import matplotlib.pyplot as plt

f = open("Controls_result.txt", "r")
Controls = f.readlines() 
PHI = []
THETA =[]
THRUST = []
cnts = []

for line in Controls:

	J = line.split(" " )
	PHI.append(float(J[0]))
	THETA.append(float(J[1]) )
	THRUST.append(float(J[2]))
	cnts.append(float(J[3]))


PHI2 = []
THETA2 = []
THRUST2 = []

for i in range( len(cnts) ):

	PHI2.append(PHI[-i])
	THETA2.append(THETA[-i])
	THRUST2.append(THRUST[-i])



plt.title("Graphs of Phi Theta and Thrust")
plt.subplot( 3,1,1)
plt.plot( cnts , PHI , label='Phi')
plt.legend()
plt.subplot( 3,1,2)
plt.plot(cnts, THETA , label='Theta')
plt.legend()
plt.subplot( 3,1,3)
plt.plot(cnts, THRUST, label='Thrust')
plt.xlabel("Number of Control instances")
plt.legend()
plt.show()


