#Run a neural network to calculate a set of 12 instrinsic parameters for an IMU
import tensorflow as tf
import math
import scipy.io as sio
import numpy as np
print("Tensorflow version " + tf.__version__)
tf.set_random_seed(0.0)


# neural network with 4 layers
#
# · · · · · · · ·           (input data, flattened GNSS and IMU data)  X [batch,7,2000]
# \x/x\x/x\x/x\x/       -- fully connected layer (relu+BN)      W1 [14000, 2000]      B1[2000]
#  · · · · · · ·                                                Y1 [batch, 2000]
#   \x/x\x/x\x/         -- fully connected layer (relu+BN)      W2 [2000, 286]      B2[286]
#    · · · · ·                                                  Y2 [batch, 286]
#     \x/x\x/           -- fully connected layer (relu+BN)      W3 [286, 48]       B3[48]
#      · · ·                                                    Y3 [batch, 48]
#       \x/             -- fully connected layer (relu+BN)      W4 [48, 12]        B4[12]
#        ·                                                      Y4 [batch, 12]

# input X: 7x2000 data vector, the last dimension (None) will index the images in the mini-batch
X = tf.placeholder(tf.float32, [None,18])
# correct parameters will go here
Y_ = tf.placeholder(tf.float32, [None,1)
# variable learning rate
lr = tf.placeholder(tf.float32)
pkeep = tf.placeholder(tf.float32)


# Neural network with three hidden layers.  More can be added by increasing depth here.
L = 18
M = 15
N = 10
O = 5


#Weights and biases initialized with small random values
W1 = tf.Variable(tf.truncated_normal([40, L], stddev=0.2))
B1 = tf.Variable(tf.constant(0.1, tf.float32, [L]))
W2 = tf.Variable(tf.truncated_normal([L, M], stddev=0.2))
B2 = tf.Variable(tf.constant(0.1, tf.float32, [M]))
W3 = tf.Variable(tf.truncated_normal([M, N], stddev=0.2))
B3 = tf.Variable(tf.constant(0.1, tf.float32, [N]))
W4 = tf.Variable(tf.truncated_normal([N, O], stddev=0.2))
B4 = tf.Variable(tf.constant(0.1, tf.float32, [O]))
W5 = tf.Variable(tf.truncated_normal([O, 1], stddev=0.2))
B5 = tf.Variable(tf.constant(0.1, tf.float32, [1]))


#The Model
Y1 = tf.nn.tanh(tf.matmul(X,W1) + B1)
Y1d = tf.nn.dropout(Y1,pkeep)
Y2 = tf.nn.tanh(tf.matmul(Y1d,W2) + B2)
Y2d = tf.nn.dropout(Y2,pkeep)
Y3 = tf.nn.tanh(tf.matmul(Y2d,W3) + B3)
Y3d = tf.nn.dropout(Y3,pkeep)
Y4 = tf.nn.tanh(tf.matmul(Y3d,W4) + B4)
Y4d = tf.nn.dropout(Y4,pkeep)
Y = (tf.matmul(Y4d,W5) + B5)



# Mean Square Loss works is the simplest option for regression problems
loss = tf.losses.mean_squared_error(Y_, Y)

# Huber Loss is more robust when dealing with data outliers above some delta
#loss = tf.losses.huber_loss(labels=Y_, predictions=Y, delta=1.0)

# Absolute difference is great if we expect all data of the same magnitude
#loss = tf.losses.absolute_difference(Y_, Y)

# Weight the loss by mini-batch size to speed up convergence
loss = tf.reduce_mean(loss)*10

#Accuracy of the trained model, between 0% (worst) and 100% (best)
#accuracy = 100 * tf.reduce_mean(1 - tf.minimum(tf.ones([12]), tf.abs((Y_-Y)/Y_)))
accuracy = 100 * (1 - tf.abs((tf.norm(Y_) - tf.norm(Y))/tf.norm(Y_)))

# Training step
# Adadelta is best if parameters have different sparsity
#train_step =tf.train.AdadeltaOptimizer(learning_rate=lr).minimize(loss)

# Adam is our default
train_step = tf.train.AdamOptimizer(lr).minimize(loss)

#Add option to save and retrain
saver = tf.train.Saver()

#Init
init = tf.global_variables_initializer()
sess = tf.Session()
sess.run(init)

#Call this function in a loop to train the model, 10 flights at a time
#This is the training data matrix

#This is the test data matrix
#mat2 = sio.loadmat('/Users/evansrnka/PycharmProjects/nn_imu/imudat2')
def training_step(update_train_data,update_test_data,dataindex,testindex):
    #Train on batches of 10

    max_learning_rate = 0.01
    min_learning_rate = 0.0001
    decay_speed = 1000
    learning_rate = min_learning_rate + (max_learning_rate - min_learning_rate) * math.exp(-testindex/decay_speed)

    if update_train_data:
        batch_X = mat1['data'][10*dataindex:10*(dataindex+1),:]
        batch_Y = mat1['levers'][10*dataindex:10*(dataindex+1),:]
        c = sess.run([loss],{X: batch_X, Y_: batch_Y, pkeep: 1.0})
        print("Epoch: " + str(testindex+1) + " Run: " + str(dataindex) + " loss:" + str(c) + " (lr:" + str(learning_rate) + ")")
        #the backpropagation training step
        sess.run(train_step, {X:batch_X, Y_: batch_Y, lr: learning_rate, pkeep: 0.85})
    if update_test_data:
        test_X = mat1['datset'][:,:]
        test_Y = mat1['levers'][:,:]
        c, predict, truth = sess.run([loss,Y,Y_],{X: test_X, Y_: test_Y, pkeep: 1.0})
        print(str(testindex+1)+": ********** TEST " + " loss:" + str(c) + " (lr:" + str(learning_rate) + ")")
#        print(np.mean(predict,0))
#        print(np.mean(truth,0))
        print(predict)
        print(truth)




#The training loop
for j in range(4):
    for L in range(15):
        #mat1 = sio.loadmat('trainData01_processed.mat')
        loadstr = 'trainData' + '%0*d'%(2,L+1) + '_processed.mat'
        #print(loadstr)
        mat1 = sio.loadmat(loadstr, squeeze_me=True)
        for k in range(10):
            training_step(True, False, k, j)
    mat1=sio.loadmat('testdata.mat')
    training_step(False, True, 0, j)

save_path = saver.save(sess, "/Users/nick/Desktop/model.ckpt")
print("Model saved in file: %s" % save_path)



