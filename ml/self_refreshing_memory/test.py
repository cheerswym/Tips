# x1_axis = np.linspace(-20,-0.001,75)
# x2_axis = np.linspace(-20,-0.001,75)

x1_axis = np.linspace(0.001,20,75)
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
        outputs = old_model(inputs)
        true_y.append(data_generator(x1_axis[m],x2_axis[n]))
        label = outputs.data[0].numpy()
        pre_y.append(label[-1])
X1 = np.array(X1)
X2 = np.array(X2)
true_y = np.array(true_y).ravel()
pre_y = np.array(pre_y).ravel()
error = np.sum(np.abs(true_y-pre_y))/len(true_y)
print('error: '+str(error))
