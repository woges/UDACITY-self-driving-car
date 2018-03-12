import os.path
import tensorflow as tf
import helper
import warnings
from distutils.version import LooseVersion
import project_tests as tests
import time
import numpy as np


# here definition of constants, etc.
IMAGE_SHAPE = (160, 576)
NUM_CLASSES = 2
TRAIN = True # True, False
RANDOM = False # True, False
TEST = True #False, True
AUGMENT = True # True, False

# Check TensorFlow Version
assert LooseVersion(tf.__version__) >= LooseVersion('1.0'), 'Please use TensorFlow version 1.0 or newer.  You are using {}'.format(tf.__version__)
print('TensorFlow Version: {}'.format(tf.__version__))

# Check for a GPU
if not tf.test.gpu_device_name():
    warnings.warn('No GPU found. Please use a GPU to train your neural network.')
else:
    print('Default GPU Device: {}'.format(tf.test.gpu_device_name()))

def load_vgg(sess, vgg_path):
    """
    Load Pretrained VGG Model into TensorFlow.
    :param sess: TensorFlow Session
    :param vgg_path: Path to vgg folder, containing "variables/" and "saved_model.pb"
    :return: Tuple of Tensors from VGG model (image_input, keep_prob, layer3_out, layer4_out, layer7_out)
    """
    #   Use tf.saved_model.loader.load to load the model and weights
    
    ###get_tensor_by_name
    
    vgg_tag = 'vgg16'
    vgg_input_tensor_name = 'image_input:0'
    vgg_keep_prob_tensor_name = 'keep_prob:0'
    vgg_layer3_out_tensor_name = 'layer3_out:0'
    vgg_layer4_out_tensor_name = 'layer4_out:0'
    vgg_layer7_out_tensor_name = 'layer7_out:0'

    #Load the saved model
    tf.saved_model.loader.load(sess,[vgg_tag], vgg_path) 
    graph = tf.get_default_graph()
    #Load variables

    input_tensor = graph.get_tensor_by_name(vgg_input_tensor_name)
    keep_prob_vgg = graph.get_tensor_by_name(vgg_keep_prob_tensor_name)
    layer3_out = graph.get_tensor_by_name(vgg_layer3_out_tensor_name)
    layer4_out = graph.get_tensor_by_name(vgg_layer4_out_tensor_name)      
    layer7_out = graph.get_tensor_by_name(vgg_layer7_out_tensor_name)

    return input_tensor, keep_prob_vgg, layer3_out, layer4_out, layer7_out

tests.test_load_vgg(load_vgg, tf)

def layers(vgg_layer3_out, vgg_layer4_out, vgg_layer7_out, num_classes, keep_prob = 1.0):
    """
    Create the layers for a fully convolutional network.  Build skip-layers using the vgg layers.
    :param vgg_layer7_out: TF Tensor for VGG Layer 3 output
    :param vgg_layer4_out: TF Tensor for VGG Layer 4 output
    :param vgg_layer3_out: TF Tensor for VGG Layer 7 output
    :param num_classes: Number of classes to classify
    :return: The Tensor for the last layer of output
    """
    #Implement function
    layer7_1x1 = tf.layers.conv2d(
                    inputs=vgg_layer7_out, 
                    filters=num_classes,
                    kernel_size=(1,1), 
                    strides=(1,1),
                    padding="same",
                    kernel_initializer=tf.truncated_normal_initializer(stddev=0.01),
                    kernel_regularizer=tf.contrib.layers.l2_regularizer(1e-3))

    layer4_1x1 = tf.layers.conv2d(
                    inputs=vgg_layer4_out, 
                    filters=num_classes,
                    kernel_size=(1,1), 
                    strides=(1,1),
                    padding="same",
                    kernel_initializer=tf.truncated_normal_initializer(stddev=0.01),
                    kernel_regularizer=tf.contrib.layers.l2_regularizer(1e-3))
    
    layer3_1x1 = tf.layers.conv2d(
                    inputs=vgg_layer3_out, 
                    filters=num_classes,
                    kernel_size=(1,1), 
                    strides=(1,1),
                    padding="same",
                    kernel_initializer=tf.truncated_normal_initializer(stddev=0.01),
                    kernel_regularizer=tf.contrib.layers.l2_regularizer(1e-3))

    layer7_deconv = tf.layers.conv2d_transpose(
                    inputs=layer7_1x1, 
                    filters=num_classes,
                    kernel_size=(4,4), 
                    strides=(2,2),
                    padding="same",
                    kernel_initializer=tf.truncated_normal_initializer(stddev=0.01),
                    kernel_regularizer=tf.contrib.layers.l2_regularizer(1e-3))

    skip_l4_l7 = tf.add(layer4_1x1,layer7_deconv)
    l4_l7_deconv = tf.layers.conv2d_transpose(
                    inputs=skip_l4_l7, 
                    filters=num_classes,
                    kernel_size=(4,4), 
                    strides=(2,2),
                    padding="same",
                    kernel_initializer=tf.truncated_normal_initializer(stddev=0.01),
                    kernel_regularizer=tf.contrib.layers.l2_regularizer(1e-3))

    skip_l3_l4_l7 = tf.add(layer3_1x1,l4_l7_deconv)
    l3_l4_l7_deconv = tf.layers.conv2d_transpose(
                    inputs=skip_l3_l4_l7, 
                    filters=num_classes,
                    kernel_size=(16,16), 
                    strides=(8,8),
                    padding="same",
                    kernel_initializer=tf.truncated_normal_initializer(stddev=0.01),
                    kernel_regularizer=tf.contrib.layers.l2_regularizer(1e-3))
  
    return l3_l4_l7_deconv
tests.test_layers(layers)

def optimize(nn_last_layer, correct_label, learning_rate_, num_classes):
    """
    Build the TensorFLow loss and optimizer operations.
    :param nn_last_layer: TF Tensor of the last layer in the neural network
    :param correct_label: TF Placeholder for the correct label image
    :param learning_rate: TF Placeholder for the learning rate
    :param num_classes: Number of classes to classify
    :return: Tuple of (logits, train_op, cross_entropy_loss)
    """
    with tf.name_scope("logits"):
        logits_ = tf.reshape(nn_last_layer, (-1, num_classes))
    labels_ = tf.reshape(correct_label, (-1, num_classes))
    with tf.name_scope("softmax"):
        soft_max = tf.nn.softmax_cross_entropy_with_logits(logits=logits_, labels=labels_)
    with tf.name_scope("xent"):
        cross_entropy_loss = tf.reduce_mean(soft_max)
    ### Using AdamOptimizer
    with tf.name_scope("train"):
        optimizer = tf.train.AdamOptimizer(learning_rate = learning_rate_).minimize(cross_entropy_loss)
    ###
        
    tf.summary.histogram("logits", logits_)
    tf.summary.histogram("softmax", soft_max)
    tf.summary.scalar("xent", cross_entropy_loss)

    return (logits_, optimizer, cross_entropy_loss)
tests.test_optimize(optimize)

def train_nn(sess, epochs, batch_size, get_batches_fn, train_op, 
             cross_entropy_loss, input_image, correct_label, keep_prob, gr_keep_prob,
             learning_rate, gr_learning_rate):
    """
    Train neural network and print out the loss during training.
    :param sess: TF Session
    :param epochs: Number of epochs
    :param batch_size: Batch size
    :param get_batches_fn: Function to get batches of training data.  Call using get_batches_fn(batch_size)
    :param train_op: TF Operation to train the neural network
    :param cross_entropy_loss: TF Tensor for the amount of loss
    :param input_image: TF Placeholder for input images
    :param correct_label: TF Placeholder for label images
    :param keep_prob: TF Placeholder for dropout keep probability
    :param learning_rate: TF Placeholder for learning rate
    """
    #start Tensorboard
    tb_file = './TB/' + time.strftime("%Y%m%d_%H%M%S")
    writer = tf.summary.FileWriter(tb_file)
    merged_nn_info = tf.summary.merge_all()
    writer.add_graph(sess.graph)
    
    print("Training...")
    print()
    for epoch_step in range(epochs):
        step_sum = 0
        loss_sum = 0       
        for images, labels in get_batches_fn(batch_size):
            step_sum +=1
            input_dict = {input_image: images, correct_label: labels, keep_prob:gr_keep_prob, learning_rate:gr_learning_rate}
            t_op,loss = sess.run([train_op, cross_entropy_loss],input_dict)            
            loss_sum +=loss
            if(step_sum%5 == 0):
                sum_info = sess.run(merged_nn_info,input_dict)
                writer.add_summary(sum_info, epoch_step+1)
        print("EPOCH {}.  Loss_mean = {:.3f}   Loss_last = {:.3f}".format(epoch_step+1, loss_sum/step_sum, loss))
        sum_info = sess.run(merged_nn_info,input_dict)
        writer.add_summary(sum_info, epoch_step+1)

    print("Training...end")
    return round((loss_sum/step_sum)*1000)/1000
tests.test_train_nn(train_nn)

#https://stackoverflow.com/questions/40467296/tensorflow-how-to-implement-hyper-parameters-random-search
def generate_random_hyperparams(lr_min, lr_max, kp_min, kp_max,ep_min, ep_max, bs_min, bs_max):
    '''generate random learning rate and keep probability'''
    # random search through log space for learning rate
    random_learning_rate = round(10**np.random.uniform(lr_min, lr_max)*2000000)/1000000
    random_keep_prob = round(np.random.uniform(kp_min, kp_max)*100)/100
    random_epochs = np.random.randint(ep_min, ep_max+1)
    random_bs = np.random.randint(bs_min, bs_max+1)
    return random_learning_rate, random_keep_prob,random_epochs,random_bs  



def run():
    data_dir = './data'
    runs_dir = './runs'
    save_file = './save/fcn_model_ckpt_' + time.strftime("%Y%m%d_%H%M%S")
    
    tests.test_for_kitti_dataset(data_dir)

    # Download pretrained vgg model
    helper.maybe_download_pretrained_vgg(data_dir)

    # OPTIONAL: Train and Inference on the cityscapes dataset instead of the Kitti dataset.
    # You'll need a GPU with at least 10 teraFLOPS to train on.
    #  https://www.cityscapes-dataset.com/
    if AUGMENT:
        helper.generate_augmented_images(os.path.join(data_dir, 'data_road/training'), IMAGE_SHAPE)

    with tf.Session() as sess:


        performance_records = {}

        # Path to vgg model
        vgg_path = os.path.join(data_dir, 'vgg')
        # Create function to get batches
        get_batches_fn = helper.gen_batch_function(os.path.join(data_dir, 'data_road/training'), IMAGE_SHAPE)

        # OPTIONAL: Augment Images for better results
        #  https://datascience.stackexchange.com/questions/5224/how-to-prepare-augment-images-for-neural-network

        #Build NN using load_vgg, layers, and optimize function

        ###
        correct_label = tf.placeholder(tf.float32, shape = [None, None, None, NUM_CLASSES])
        learning_rate = tf.placeholder(tf.float32)
        keep_prob = tf.placeholder(tf.float32)
        
        input_tensor, keep_prob, layer3_out, layer4_out, layer7_out = load_vgg(sess, vgg_path)
        final_layer = layers(layer3_out, layer4_out, layer7_out, NUM_CLASSES, keep_prob)
        logits, train_op, cross_entropy_loss = optimize(final_layer, correct_label, learning_rate, NUM_CLASSES)
        ###
        #Train NN using the train_nn function

        print("sess.run")
        if(TRAIN):
            if(RANDOM):
                gr_learning_rate, gr_keep_prob, gr_epochs, gr_batch_size = generate_random_hyperparams(-5, -4, 0.7, 0.9, 20, 45, 4, 8)
                nb_set = 10
            else:
                gr_learning_rate = 0.000055
                gr_keep_prob = 0.89
                gr_epochs = 43
                gr_batch_size = 5
                nb_set = 1
            for i in range(nb_set):
                sess.run(tf.global_variables_initializer())
                saver = tf.train.Saver()
                print("gr_epochs: ", gr_epochs, "  gr_batch_size: ", gr_batch_size, "  gr_keep_prob: ", gr_keep_prob, "  gr_learning_rate: ", gr_learning_rate)
                loss_mean = train_nn(sess, gr_epochs, gr_batch_size, get_batches_fn, train_op, cross_entropy_loss,
                                     input_tensor,correct_label, keep_prob, gr_keep_prob, learning_rate, gr_learning_rate)
                ##save trained network
                saver.save(sess, save_file)
                performance_records[(gr_epochs, gr_batch_size, gr_keep_prob, gr_learning_rate)] = loss_mean
                print("gr_epochs: ", gr_epochs, "  gr_batch_size: ", gr_batch_size, "  gr_keep_prob: ", gr_keep_prob, "  gr_learning_rate: ", gr_learning_rate,  "  loss_mean: ", loss_mean)
        # Save inference data using helper.save_inference_samples
        #  helper.save_inference_samples(runs_dir, data_dir, sess, image_shape, logits, keep_prob, input_image)
        if(TEST):
            saver = tf.train.Saver()
            saver.restore(sess, tf.train.latest_checkpoint('save'))
            helper.save_inference_samples(runs_dir, data_dir, sess, IMAGE_SHAPE, logits, keep_prob, input_tensor)
        # OPTIONAL: Apply the trained model to a video
        #writer.add:graph(sess.graph)

if __name__ == '__main__':
    run()




