import pickle
import os
import numpy as np
import theano
import lasagne

log_dir = os.path.join(os.getcwd(),'data')

with open(os.path.join(log_dir,'policy.pkl'), 'rb') as input:
	policy = pickle.load(input)

h0w = policy._cached_params[()][0].get_value()
h0b = policy._cached_params[()][1].get_value()
h1w = policy._cached_params[()][2].get_value()
h1b = policy._cached_params[()][3].get_value()
ow = policy._cached_params[()][4].get_value()
ob = policy._cached_params[()][5].get_value()

ologstd = policy._cached_params[()][6].get_value()

obser = np.linspace(0,17,18).reshape(18,1)

h0_out = np.tanh(np.matmul(obser.T, h0w) + h0b)
h1_out = np.tanh(np.matmul(h0_out, h1w) + h1b)
out = np.matmul(h1_out, ow) + ob

# for testing
policy.get_action(obser)
# same as
l_out = policy._l_mean
l_in = policy._mean_network.input_layer
f = theano.function([l_in.input_var], lasagne.layers.get_output(l_out))
print(out)
print(f(obser.T))