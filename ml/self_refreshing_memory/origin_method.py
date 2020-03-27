import numpy as np
from collections import namedtuple
import torch
import torch.nn.functional as F
from torch.autograd import Variable
import torch.nn as nn
from sklearn import preprocessing
from sklearn.decomposition import PCA
import torch.optim as optim
import time
import random

random.seed(0)
FloatTensor = torch.FloatTensor
LongTensor = torch.LongTensor
ByteTensor = torch.ByteTensor
Tensor = FloatTensor

def data_generator(x1, x2):
    return np.sin(x1)/x1+x2/10
Transition = namedtuple('Transition',
                        ('state', 'label'))

class ReplayMemory(object):
    def __init__(self, capacity):
        self.capacity = capacity
        self.memory = []
        self.position = 0

    def push(self, *args):
        """Saves a transition."""
        if len(self.memory) < self.capacity:
            self.memory.append(None)
        self.memory[self.position] = Transition(*args)
        self.position = (self.position + 1) % self.capacity

    def sample(self, batch_size):
        return random.sample(self.memory, batch_size)

    def __len__(self):
        return len(self.memory)

"""  MLP  """
class Net(nn.Module):
    def __init__(self):
        super(Net,self).__init__()
        self.fc1 = nn.Linear(2, 100)
        self.fc2 = nn.Linear(100, 3)
    def forward(self, x):
        x = torch.sigmoid(self.fc1(x))
        return self.fc2(x)

print("begin run...")
data = ReplayMemory(3000)
for i in range(3000):
    x1 = 20.0*np.random.rand(1)-20.0
    x2 = 20.0*np.random.rand(1)-20.0
    x = np.array([x1[0], x2[0]])
    y = data_generator(x1[0], x2[0])
    label = np.array([x1[0], x2[0], y])
    data.push(Tensor([x]),Tensor([label]))

model = Net()
criterion = nn.MSELoss()
optimizer = optim.Adam(model.parameters(), lr=0.001, betas=(0.9, 0.999), eps=1e-08, weight_decay=0)
tic = time.time()
for i in range(60000):
    # get the inputs
    transitions = data.sample(25)
    batch = Transition(*zip(*transitions))
    state_batch = Variable(torch.cat(batch.state))
    label_batch = Variable(torch.cat(batch.label))

    # zero the parameter gradients
    optimizer.zero_grad()

    # forward + backward + optimize
    outputs = model(state_batch)

    loss = criterion(outputs, label_batch)
    loss.backward()
    optimizer.step()
toc = time.time()
print("the time cost is:"+str(toc-tic))
print('Finished Training')

x1_axis = np.linspace(-20,-0.001,75)
x2_axis = np.linspace(-20,-0.001,75)



X1 = []
X2 = []

true_y = []
pre_y = []
for m in range(len(x1_axis)):
    for n in range(len(x2_axis)):
        X1.append(x1_axis[m])
        X2.append(x2_axis[n])
        x = np.array([[x1_axis[m], x2_axis[n]]])
        inputs = Variable(torch.Tensor(x))
        outputs = model(inputs)
        true_y.append(data_generator(x1_axis[m],x2_axis[n]))
        label = outputs.data[0].numpy()
        pre_y.append(label[-1])
X1 = np.array(X1)
X2 = np.array(X2)
true_y = np.array(true_y).ravel()
pre_y = np.array(pre_y).ravel()
error = np.sum(np.abs(true_y-pre_y))/len(true_y)
print('error: '+str(error))
