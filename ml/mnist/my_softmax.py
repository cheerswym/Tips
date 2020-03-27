import input_data
# import tensorflow as tf
import tensorflow.compat.v1 as tf
tf.disable_v2_behavior()


mnist = input_data.read_data_sets("MNIST_data/", one_hot=True)



print ('输入数据:',mnist.train.images)
print ('输入数据打印shape:',mnist.train.images.shape)
import pylab
im = mnist.train.images[1]
im = im.reshape(-1,28)
pylab.imshow(im)
pylab.show()
print ('输入数据打印shape:',mnist.test.images.shape)
print ('输入数据打印shape:',mnist.validation.images.shape)



x = tf.placeholder("float", [None, 784])
W = tf.Variable(tf.zeros([784,10]))
b = tf.Variable(tf.zeros([10]))

y = tf.nn.softmax(tf.matmul(x,W) + b)

y_ = tf.placeholder("float", [None,10])

# cross entropy 一般用于整体计算
cross_entropy = -tf.reduce_sum(y_*tf.log(y))

# I think ("train_step") can be understand as the function pointer
train_step = tf.train.GradientDescentOptimizer(0.01).minimize(cross_entropy)

init = tf.initialize_all_variables()
sess = tf.Session()
sess.run(init)

for i in range(1000):
	batch_xs, batch_ys = mnist.train.next_batch(100)
	sess.run(train_step, feed_dict={x: batch_xs, y_: batch_ys})

# Compare one by one.
correct_prediction = tf.equal(tf.argmax(y,1), tf.argmax(y_,1))

accuracy = tf.reduce_mean(tf.cast(correct_prediction, "float"))

print(sess.run(accuracy, feed_dict={x: mnist.test.images, y_: mnist.test.labels}))