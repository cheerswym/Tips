import copy
import time
new_model = copy.deepcopy(model)
new_optimizer = optim.Adam(new_model.parameters(), lr=0.001, betas=(0.9, 0.999), eps=1e-08, weight_decay=0)
old_model = copy.deepcopy(model)
old_optimizer = optim.Adam(old_model.parameters(), lr=0.001, betas=(0.9, 0.999), eps=1e-08, weight_decay=0)
tc = time.time()
def generate_new(batch_size):
    test_x = 40*np.random.rand(batch_size,2)-20
    for i in range(100):
        inputs = Variable(torch.Tensor(test_x))
        outputs = new_model(inputs)
        label = outputs.data.numpy()
        test_x[:,0] = label[:,0]
        test_x[:,1] = label[:,1]
    X = test_x
    Y = label
    return Tensor(X),Tensor(label)

def generate_old(batch_size):
    test_x = 40*np.random.rand(batch_size,2)-20
    for i in range(100):
        inputs = Variable(torch.Tensor(test_x))
        outputs = old_model(inputs)
        label = outputs.data.numpy()
        test_x[:,0] = label[:,0]
        test_x[:,1] = label[:,1]
    X = test_x
    Y = label
    return Tensor(X),Tensor(label)
    
data = ReplayMemory(3000)
for i in range(3000):
    x1 = 20.0*np.random.rand(1)-0.0
    x2 = 20.0*np.random.rand(1)-20.0
    x = np.array([x1[0], x2[0]])
    y = data_generator(x1[0], x2[0])
    label = np.array([x1[0], x2[0], y])
    data.push(Tensor([x]),Tensor([label]))
    
tic = time.time()
for i in range(60000):
    # get the inputs
    transitions = data.sample(10)
    batch = Transition(*zip(*transitions))
    extern_state = torch.cat(batch.state)
    extern_label = torch.cat(batch.label)
#     new_active_state, new_active_label = generate_new(10)
    
#     state_batch = Variable(extern_state)
#     label_batch = Variable(extern_label)    
    
    state_batch = Variable(torch.cat((extern_state,new_active_state),0))
    label_batch = Variable(torch.cat((extern_label,new_active_label),0))

    # zero the parameter gradients
    old_optimizer.zero_grad()

    # forward + backward + optimize
    outputs = old_model(state_batch)

    loss = criterion(outputs, label_batch)
    loss.backward()
    old_optimizer.step()
    
    
    
    old_active_state, old_active_label = generate_old(10)
    old_state_batch = Variable(old_active_state)
    old_label_batch = Variable(old_active_label)
    new_optimizer.zero_grad()
    old_outputs = new_model(old_state_batch)
    loss = criterion(old_outputs, old_label_batch)
    loss.backward()
    new_optimizer.step()    
    
toc = time.time()
print("the time cost is:"+str(toc-tic))
print('Finished Training')
