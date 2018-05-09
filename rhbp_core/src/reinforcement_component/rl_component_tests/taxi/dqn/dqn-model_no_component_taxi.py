import gym
import numpy as np
import random
import tensorflow as tf

# Helper functions


from tensorflow.contrib import slim

env = gym.make('Taxi-v2')
env.seed(2)

# class for revisiting experiences
class experience_buffer():
    def __init__(self, buffer_size=10000):
        self.buffer = []
        self.buffer_size = buffer_size
        self.counter = 0
        self.print_steps = False
        random.seed(0)
    # add a new experience
    def add(self, experience):
        if self.print_steps:
            print(self.counter,np.argmax(experience[0,0]),np.argmax(experience[0,3]),experience[0,1],experience[0,2])
        self.counter +=1
        if len(self.buffer) + len(experience) >= self.buffer_size:
            self.buffer[0:(len(experience) + len(self.buffer)) - self.buffer_size] = []
        self.buffer.extend(experience)
    # get a random sample of the buffer
    def sample(self, size):
        sample = np.reshape(np.array(random.sample(self.buffer, size)), [size, 5])
        if self.print_steps:
            print("train",np.argmax(sample[10][0]),np.argmax(sample[10][3]),sample[10][1],sample[10][2])
        return sample


def updateTargetGraph(tfVars, tau):
    """
    returns a list of operations coming from the trainable variables
    :param tfVars: 
    :param tau: 
    :return: 
    """
    total_vars = len(tfVars)
    op_holder = []
    for idx, var in enumerate(tfVars[0:total_vars / 2]):
        op_holder.append(tfVars[idx + total_vars / 2].assign(
            (var.value() * tau) + ((1 - tau) * tfVars[idx + total_vars / 2].value())))
    return op_holder


def updateTarget(op_holder, sess):
    """
    run each operation in op_holder 
    :param op_holder: 
    :param sess: 
    :return: 
    """
    for op in op_holder:
        sess.run(op)


# Implementing the network itself
class Q_Network():
    def __init__(self):
        tf.set_random_seed(0)
        # defining_parameters
        number_outputs = 6
        number_inputs = 500
        # These lines establish the feed-forward part of the network used to choose actions
        # these describe the observation (input),
        self.inputs = tf.placeholder(shape=[None, number_inputs], dtype=tf.float32)
        #self.inputs = tf.cast(self.inputs,tf.float32)
        self.Temp = tf.placeholder(shape=None, dtype=tf.float32)
        self.keep_per = tf.placeholder(shape=None, dtype=tf.float32)

        # the layers that define the nn
        #one_hot_inputs = tf.one_hot(self.inputs,number_inputs,dtype=tf.float32)
        hidden = slim.fully_connected(self.inputs, 64, activation_fn=tf.nn.tanh, biases_initializer=None)
        # drop tensors out and scales others by probability of self.keep_per
        hidden = slim.dropout(hidden, self.keep_per)
        # layer for computing the q_values
        self.Q_out = slim.fully_connected(hidden, number_outputs, activation_fn=None, biases_initializer=None)
        # prediction is highest q-value
        self.predict = tf.argmax(self.Q_out, 1)
        # compute the softmax activations.
        self.Q_dist = tf.nn.softmax(self.Q_out / self.Temp)

        # Below we obtain the loss by taking the sum of squares difference between the target and prediction Q values.
        self.actions = tf.placeholder(shape=[None], dtype=tf.int32)
        self.actions_onehot = tf.one_hot(self.actions, number_outputs, dtype=tf.float32)

        self.Q = tf.reduce_sum(tf.multiply(self.Q_out, self.actions_onehot), reduction_indices=1)
        #tf.mul
        self.nextQ = tf.placeholder(shape=[None], dtype=tf.float32)
        loss = tf.reduce_sum(tf.square(self.nextQ - self.Q))
        # updating the weights of the model to minimize the loss function
        trainer = tf.train.GradientDescentOptimizer(learning_rate=0.0005)
        self.updateModel = trainer.minimize(loss)

def one_hot(s,size):
    return np.identity(size)[s:s + 1][0].tolist()

def decode(i):
    out = []
    out.append(i % 4)  # row
    i = i // 4
    out.append(i % 5)  # col
    i = i // 5
    out.append(i % 5)  # passloc
    i = i // 5
    out.append(i)  # destination
    assert 0 <= i < 5
    return reversed(out)

def get_array(s):
    return one_hot(s,500)
    row, col, passenger, dest = decode(s)
    array = one_hot(row,5)
    array.extend(one_hot(col,5))
    array.extend(one_hot(passenger,5))
    array.extend(one_hot(dest,4))
    return array
# Train the model

# Set learning parameters
exploration = "e-greedy" #Exploration method. Choose between: greedy, random, e-greedy, boltzmann, bayesian.
y = .99 #Discount factor.
num_episodes = 10000 #Total number of episodes to train network for.
tau = 0.001 #Amount to update target network at each step.
batch_size = 32 #Size of training batch
startE = 1 #Starting chance of random action
endE = 0.1 #Final chance of random action
anneling_steps = 400000 #How many steps of training to reduce startE to endE.
pre_train_steps = 32 # Number of steps used before training updates begin.

tf.set_random_seed(0)
np.random.seed(2)
print_steps = False

tf.reset_default_graph()

# initialize two networks. q-network and target q-network
q_net = Q_Network()
target_net = Q_Network()

init = tf.global_variables_initializer()
#returns all variables created with trainable=True
trainables = tf.trainable_variables()
# create target operations
targetOps = updateTargetGraph(trainables, tau)
# buffer class for experience learning
myBuffer = experience_buffer()

# create lists to contain total rewards and steps per episode
jList = []
jMeans = []
rList = []
rMeans = []
with tf.Session() as sess:
    # init all variables
    sess.run(init)
    # run target operations
    updateTarget(targetOps, sess)
    e = startE
    stepDrop = (startE - endE) / anneling_steps
    total_steps = 0
    # iteratate number of episodes
    for i in range(num_episodes):
        # reset environment in beginning of each step
        s = env.reset()
        #print(get_array(s))
        #s = [s,0]
        #s = s
        #s=tf.one_hot(s,500,dtype=tf.int32)
        #s = get_array(s)
        #print(s)
        s = get_array(s)
        #s = get_array(s)
        #print("s",sess.run(s))
        #print(s)
        rAll = 0
        d = False
        j = 0
        # episodes end after 999 steps or when failed
        while j < 999:
            j += 1
            # depending which strategy for exploration choose a action from the q-network
            if exploration == "greedy":
                # Choose an action with the maximum expected value.
                a, allQ = sess.run([q_net.predict, q_net.Q_out], feed_dict={q_net.inputs: [s], q_net.keep_per: 1.0})
                a = a[0]
            if exploration == "random":
                # Choose an action randomly.
                a = env.action_space.sample()
            if exploration == "e-greedy":
                #print([s])
                #print(len(s))
                #print(np.array(s).shape)
                # Choose an action by greedily (with e chance of random action) from the Q-network
                random_value = np.random.rand(1)
                if  random_value < e or total_steps < pre_train_steps:
                    #a = env.action_space.sample()
                    a = np.random.randint(6)
                    #print("random",a,random_value,e)
                else:
                    a, allQ = sess.run([q_net.predict, q_net.Q_out], feed_dict={q_net.inputs: [s], q_net.keep_per: 1.0})
                    a = a[0]
                    #print(allQ)

            if exploration == "boltzmann":
                # Choose an action probabilistically, with weights relative to the Q-values.
                Q_d, allQ = sess.run([q_net.Q_dist, q_net.Q_out],
                                     feed_dict={q_net.inputs: [s], q_net.Temp: e, q_net.keep_per: 1.0})
                a = np.random.choice(Q_d[0], p=Q_d[0])
                a = np.argmax(Q_d[0] == a)
            if exploration == "bayesian":
                # Choose an action using a sample from a dropout approximation of a bayesian q-network.
                a, allQ = sess.run([q_net.predict, q_net.Q_out],
                                   feed_dict={q_net.inputs: [s], q_net.keep_per: (1 - e) + 0.1})
                a = a[0]

            # Get new state and reward from environment
            s1, r, d, _ = env.step(a)
            s1 = get_array(s1)
            if print_steps:
                print(total_steps,np.argmax(s),np.argmax(s1),r,e)
            #s1 = tf.one_hot(s1,500,dtype=tf.int32)
            #s1 = s1
            # add train_tuple into buffer
            myBuffer.add(np.reshape(np.array([s, a, r, s1, d]), [1, 5]))

            if e > endE and total_steps > pre_train_steps:
                e -= stepDrop

            # train the model . key of algorithm. using Double DQN
            # train only if the steps are greater than the pre_train_steps and only every 5 episodes
            if total_steps > pre_train_steps and total_steps % 5 == 0:
                # We use Double-DQN training algorithm
                # get sample of buffer for training
                trainBatch = myBuffer.sample(batch_size)
                if print_steps:
                    print(total_steps, "train",np.argmax(trainBatch[10]))
                # feed resulting state and keep prob of 1 to predict action
                Q1 = sess.run(q_net.predict, feed_dict={q_net.inputs: np.vstack(trainBatch[:, 3]), q_net.keep_per: 1.0})

                # get q-values of target network with the resulting state
                Q2 = sess.run(target_net.Q_out,
                              feed_dict={target_net.inputs: np.vstack(trainBatch[:, 3]), target_net.keep_per: 1.0})
                # multiplier to add if the episode ended
                # makes reward 0 if episode ended. simulation specific
                end_multiplier = -(trainBatch[:, 4] - 1)
                #print(trainBatch[:,4],end_multiplier)
                # target-q-values of batch for choosing prediction of q-network
                #print(Q1,Q2)
                doubleQ = Q2[range(batch_size), Q1] # target_q-values for the q-net predicted action
                # target q value calculation according to q-learning
                targetQ = trainBatch[:, 2] + (y * doubleQ )#TODO add maybe here again doubleQ * endmultiplier
                # update the q-network model by giving the target-q-values, the input states and the chosen actions
                _ = sess.run(q_net.updateModel,
                             feed_dict={q_net.inputs: np.vstack(trainBatch[:, 0]), q_net.nextQ: targetQ,
                                        q_net.keep_per: 1.0, q_net.actions: trainBatch[:, 1]})
                # update the target network
                updateTarget(targetOps, sess)
            # save rewards and get new state
            rAll += r
            s = s1
            total_steps += 1
            if d == True:
                break
        # save rewards for printing
        jList.append(j)
        rList.append(rAll)
        # print results
        if i % 100 == 0 and i != 0:
            r_mean = np.mean(rList[-100:])
            j_mean = np.mean(jList[-100:])
            if exploration == 'e-greedy':
                print "Mean Reward: " + str(r_mean) + " Total Steps: " + str(total_steps) + " e: " + str(e)
            if exploration == 'boltzmann':
                print "Mean Reward: " + str(r_mean) + " Total Steps: " + str(total_steps) + " t: " + str(e)
            if exploration == 'bayesian':
                print "Mean Reward: " + str(r_mean) + " Total Steps: " + str(total_steps) + " p: " + str(e)
            if exploration == 'random' or exploration == 'greedy':
                print "Mean Reward: " + str(r_mean) + " Total Steps: " + str(total_steps)
            rMeans.append(r_mean)
            jMeans.append(j_mean)
print "Percent of succesful episodes: " + str(sum(rList) / num_episodes) + "%"