# -*- coding: utf-8 -*-
import numpy as np
x=np.array([2])
# 求f = x - cosx 当x=？, f=0  ---->  求min 0.5 * (f) ^ {2}
#SGD

for i in range(10000):
    x = x - (x - np.cos(x)) * (1 + np.sin(x)) * 0.01
    print(x - np.cos(x))

    print(x)

 

# 梯度投影法

A = np.array([[1.1]])

# 投影矩阵（等是约束）
P = np.eye(1) - np.dot(np.dot(A, (np.linalg.inv(np.dot(A.T, A) + 0.1))), A.T)
print(P)

for i in range(10000):

    x = x - P * (x - np.cos(x)) * (1 + np.sin(x)) * 0.01

    print(x - np.cos(x))

    print(x)