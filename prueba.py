from scipy import signal
import matplotlib.pyplot as plt
import numpy as np
import math as m
from control.matlab import *
from os import system

system('cls')

# Función de transferencia del proceso
k1 = 62.5
k2 = 12.9
a = 0.03271
b = 0.0002297
P = tf([k1,k2],[1,a,b])
print(P)

# Especificaciones de diseño
Mp = 10
ep = m.sqrt(((m.log(Mp/100))**2)/(m.pi**2+((m.log(Mp/100))**2)))
tss = 400
wn = 4/(ep*tss)
beta = a/(ep*wn) - 2

# Calculando los polos deseados
s1 = complex(-ep*wn,wn*m.sqrt(1-ep**2))
s2 = complex(-ep*wn,-wn*m.sqrt(1-ep**2))
s3 = -beta*ep*wn
Sd1 = [s1,s2,s3]
Pds = np.poly(Sd1)

# Cálculo de los parámetros del controlador
Kc = ((2*beta*ep**2+1)*wn**2-b)/k1
Ki = (beta*ep*wn**3)/k1
ti = Kc/Ki
td = 0
numc = [Kc,Ki]
denc = [1,0]
C = tf(numc,denc)

# Lazo de control cerrado
H = feedback(C*P,1)
print(H)

Hi = stepinfo(H)

y,t = step(H)
plt.plot(t,y)
plt.xlabel('Time [s]')
plt.ylabel('Amplitud')
plt.title('Step response')
plt.grid()
plt.show()