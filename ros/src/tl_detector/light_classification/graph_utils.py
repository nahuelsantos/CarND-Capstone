import tensorflow as tf
from tensorflow.python.framework import graph_util
from config import *
import squeezeNet
import keras.backend as K

# Used to load the tensorflow image dimension handling in keras backend
K.set_image_dim_ordering('tf')

def freeze_net(model_name):

    model = SqueezeNet(3, (IMAGE_HEIGHT, IMAGE_WIDTH, 3))
    model.load_weights("Models/{}.hdf5".format(model_name))
    # Define session and Keras configuration elements
    sess = K.get_session()

    graph = sess.graph
    input_graph_def = graph.as_graph_def()

    with sess.as_default():
        output_node_names = "softmax/Softmax"

        # Use the tensorflow graph util to convert variables into constants. Input ares:
        # sess: the current session is required for current weights retrieval
        # input_graph_def: the graph_def is needed to get all nodes structure
        # output_node_names.split(","): names are requred to select the usefull nodes only

        output_graph_def = graph_util.convert_variables_to_constants(sess, input_graph_def, output_node_names.split(","))
        
        with tf.gfile.GFile('Models/{}.frozen.pb'.format(model_name), "wb") as f:
            f.write(output_graph_def.SerializeToString())
        print("%d ops in the final graph." % len(output_graph_def.node))

def load_graph(graph_file, use_xla=False):
    jit_level = 0
    config = tf.ConfigProto()
    if use_xla:
        jit_level = tf.OptimizerOptions.ON_1
        config.graph_options.optimizer_options.global_jit_level = jit_level

    with tf.Session(graph=tf.Graph(), config=config) as sess:
        gd = tf.GraphDef()
        with tf.gfile.Open(graph_file, 'rb') as f:
            data = f.read()
            gd.ParseFromString(data)
        tf.import_graph_def(gd, name='')
        ops = sess.graph.get_operations()
        n_ops = len(ops)
        return sess.graph, ops


if __name__ == '__main__':
    freeze_net(squeezeNet_sim)
    freeze_net(squeezeNet_real)

