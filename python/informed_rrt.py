import numpy as np
import matplotlib.pyplot as plt
from random import random
 
start=[100,120]
goal=[100,80]
tempcmin=[start[0]-goal[0],start[1]-goal[1]]
 
cmin=np.linalg.norm(tempcmin,ord=2)
cbest=120
plt.figure()
while (cbest>=cmin):
    x_center=np.matrix([(start[0]+goal[0])/2.0,(start[1]+goal[1])/2.0,0])
    x_cneter0=np.transpose(x_center)
    a_1=np.matrix([[(goal[0]-start[0])/cmin],[(goal[1]-start[1])/cmin],[0]])
    id_t=np.matrix([1.0,0,0])
    M=np.dot(a_1,id_t)
    U,S,Vh=np.linalg.svd(M)
    C=np.dot(np.dot(U,np.diag([1,1,np.dot(np.linalg.det(U),np.linalg.det((np.transpose(Vh))))])),Vh)
    r=[cbest/2.0,np.sqrt(cbest**2-cmin**2)/2.0,np.sqrt(cbest**2-cmin**2)/2.0]
    L=np.diag(r)
 
    a=random()
    b=random()
    if (b<a):
        tmp=b
        b=a
        a=tmp
 
    x_ball=np.array([[b*np.cos(2*np.pi*a/b)],[b*np.sin(2*np.pi*a/b)],[0]])
    randpoint=np.dot(np.dot(C,L),x_ball)+x_cneter0
    cbest=cbest-random()*0.05
 
    plt.plot(randpoint[0],randpoint[1],'.')
    plt.hold("on")
    plt.grid("on")
    plt.show()
 
plt.show()
