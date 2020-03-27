import matplotlib.pyplot as plt
import tensorflow.compat.v1 as tf
tf.disable_v2_behavior()
import numpy as np
import math

NSAMPLE = 1000
x_data = np.random.uniform(-10.5, 10.5, size=(1, NSAMPLE)).T
r_data = np.random.normal(size=(NSAMPLE, 1))
y_data = np.cos(0.75 * x_data)* 7.0 + x_data * 0.5 + r_data

x = tf.placeholder(dtype=tf.float32, shape=[None,1])
y = tf.placeholder(dtype=tf.float32, shape=[None,1])
hidden = tf.layers.dense(x, units=20, activation=tf.nn.tanh)
# hidden1 = tf.layers.dense(hidden, units=20, activation=tf.nn.tanh)
y_out = tf.layers.dense(hidden, units=1, activation=None)

# loss = tf.nn.l2_loss(y_out - y)
loss = tf.losses.mean_squared_error(y_out, y)
# c = tf.square(y_out - y)
# loss = tf.reduce_mean(c) 

train_step = tf.train.AdamOptimizer(learning_rate=0.1).minimize(loss)

sess = tf.Session()
sess.run(tf.global_variables_initializer())

NEPOCH = 2000
for i in range(NEPOCH):
	sess.run(train_step, feed_dict={x: x_data, y: y_data})

x_test = np.arange(-10.5, 10.5, 0.1)
x_test = np.reshape(x_test, newshape=(-1, 1))
# x_test = np.random.uniform(-10.5, 10.5, size=(1, NSAMPLE)).T
y_test = sess.run(y_out, feed_dict={x: x_test})




plt.figure(figsize=(8,8))
plt.plot(x_data, y_data, 'ro', alpha = 0.3)
plt.plot(x_test, y_test, 'b*', alpha = 0.3)
plt.show()

