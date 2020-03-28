#!/usr/bin/env python
# coding: utf-8

# In[ ]:


#get_ipython().system('pip install gnn')


# In[ ]:


# import tensorflow as tf
import tensorflow.compat.v1 as tf
tf.disable_v2_behavior()

import numpy as np
import gnn.gnn_utils as gnn_utils


# In[ ]:


# reading train, validation dataset
data_path = "./data"
set_name = "sub_15_7_200"
############# training set ################
inp, arcnode, nodegraph, nodein, labels, _ = gnn_utils.set_load_general(data_path, "train", set_name=set_name)
inp = [a[:, 1:] for a in inp]
############ validation set #############
inp_val, arcnode_val, nodegraph_val, nodein_val, labels_val, _ = gnn_utils.set_load_general(data_path, "validation", set_name=set_name)
inp_val = [a[:, 1:] for a in inp_val]


# In[ ]:


input_dim = len(inp[0][0])
state_dim = 10
output_dim = 2
state_threshold = 0.001
max_iter = 50


# In[ ]:


def f_w(inp):
    with tf.variable_scope('State_net'):
        layer1 = tf.layers.dense(inp, 5, activation=tf.nn.sigmoid)
        layer2 = tf.layers.dense(layer1, state_dim, activation=tf.nn.sigmoid)
        return layer2


# In[ ]:


def g_w(inp):
    with tf.variable_scope('Output_net'):
        layer1 = tf.layers.dense(inp, 5, activation=tf.nn.sigmoid)
        layer2 = tf.layers.dense(layer1, output_dim, activation=None)
        return layer2


# In[ ]:


#init input placeholder
tf.reset_default_graph()
comp_inp = tf.placeholder(tf.float32, shape=(None, input_dim), name="input")
y = tf.placeholder(tf.float32, shape=(None, output_dim), name="target")

# state(t) & state(t-1)
state = tf.placeholder(tf.float32, shape=(None, state_dim), name="state")
state_old = tf.placeholder(tf.float32, shape=(None, state_dim), name="old_state")

# arch-node conversion matrix
ArcNode = tf.sparse_placeholder(tf.float32, name="ArcNode")


# In[ ]:


def convergence(a, state, old_state, k):
    with tf.variable_scope('Convergence'):
        # assign current state to old state
        old_state = state
        
        # 获取子结点上一个时刻的状态
        # grub states of neighboring node 
        gat = tf.gather(old_state, tf.cast(a[:, 0], tf.int32))
        
        # 去除第一列，即子结点的id
        # slice to consider only label of the node and that of it's neighbor 
        # sl = tf.slice(a, [0, 1], [tf.shape(a)[0], tf.shape(a)[1] - 1])
        # equivalent code
        sl = a[:, 1:]
        
        # 将子结点上一个时刻的状态放到最后一列
        # concat with retrieved state
        inp = tf.concat([sl, gat], axis=1)

        # evaluate next state and multiply by the arch-node conversion matrix to obtain per-node states
        #计算子结点对父结点状态的贡献
        layer1 = f_w(inp)
        #聚合子结点对父结点状态的贡献，得到当前时刻的父结点的状态
        state = tf.sparse_tensor_dense_matmul(ArcNode, layer1)

        # update the iteration counter
        k = k + 1
    return a, state, old_state, k


# In[ ]:


def condition(a, state, old_state, k):
    # evaluate condition on the convergence of the state
    with tf.variable_scope('condition'):
        # 检查当前状态和上一个时刻的状态的欧式距离是否小于阈值
        # evaluate distance by state(t) and state(t-1)
        outDistance = tf.sqrt(tf.reduce_sum(tf.square(tf.subtract(state, old_state)), 1) + 1e-10)
        # vector showing item converged or not (given a certain threshold)
        checkDistanceVec = tf.greater(outDistance, state_threshold)
        
        c1 = tf.reduce_any(checkDistanceVec)
        
        # 是否达到最大迭代次数
        c2 = tf.less(k, max_iter)

    return tf.logical_and(c1, c2)


# In[ ]:


# 迭代计算，直到状态达到稳定状态
# compute state
with tf.variable_scope('Loop'):
    k = tf.constant(0)
    res, st, old_st, num = tf.while_loop(condition, convergence,
                                         [comp_inp, state, state_old, k])
    # 计算结点的output
    out = g_w(st)


# In[ ]:


loss = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits_v2(labels=y, logits=out))
accuracy = tf.reduce_mean(tf.cast(tf.equal(tf.argmax(out, 1), tf.argmax(y, 1)), dtype=tf.float32))
optimizer = tf.train.AdamOptimizer(0.001)
grads = optimizer.compute_gradients(loss)
train_op = optimizer.apply_gradients(grads, name='train_op')


# In[ ]:


###train model####
num_epoch = 5000
# 训练集placeholder输入
arcnode_train = tf.SparseTensorValue(indices=arcnode[0].indices, values=arcnode[0].values,
                                    dense_shape=arcnode[0].dense_shape)
fd_train = {comp_inp: inp[0], state: np.zeros((arcnode[0].dense_shape[0], state_dim)),
          state_old: np.ones((arcnode[0].dense_shape[0], state_dim)),
          ArcNode: arcnode_train, y: labels}
#验证集placeholder输入
arcnode_valid = tf.SparseTensorValue(indices=arcnode_val[0].indices, values=arcnode_val[0].values,
                                dense_shape=arcnode_val[0].dense_shape)
fd_valid = {comp_inp: inp_val[0], state: np.zeros((arcnode_val[0].dense_shape[0], state_dim)),
      state_old: np.ones((arcnode_val[0].dense_shape[0], state_dim)),
      ArcNode: arcnode_valid, y: labels_val}

with tf.Session() as sess:
    sess.run(tf.global_variables_initializer())
    sess.run(tf.local_variables_initializer())
    for i in range(0, num_epoch):
        _, loss_val, accuracy_val = sess.run(
                    [train_op, loss, accuracy],
                    feed_dict=fd_train)
        if i % 100 == 0:

            loss_valid_val, accuracy_valid_val = sess.run(
                    [loss, accuracy],
                    feed_dict=fd_valid)
            print("iter %s\t training loss: %s,\t training accuracy: %s,\t validation loss: %s,\t validation accuracy: %s" % 
                  (i, loss_val, accuracy_val, loss_valid_val, accuracy_valid_val))


# In[ ]:




