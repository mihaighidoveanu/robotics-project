import argparse
import cv2
import numpy as np
import re
import os
import errno
import pickle

from inference import Network

def get_args():
    '''
    Gets the arguments from the command line.
    '''
    parser = argparse.ArgumentParser("Basic Edge App with Inference Engine")
    # -- Create the descriptions for the commands

    c_desc = "CPU extension file location, if applicable"
    d_desc = "Device, if not CPU (GPU, FPGA, MYRIAD)"
    #i_desc = "The location of the input image"

    # -- Add required and optional groups
    parser._action_groups.pop()
    required = parser.add_argument_group('required arguments')
    optional = parser.add_argument_group('optional arguments')

    # -- Create the arguments
    #required.add_argument("-i", help=i_desc, required=True)
    optional.add_argument("-c", help=c_desc, default=None)
    optional.add_argument("-d", help=d_desc, default="CPU")
    args = parser.parse_args()

    return args


def get_mask(processed_output):
    '''
    Given an input image size and processed output for a semantic mask,
    returns a masks able to be combined with the original image.
    '''
    # Create an empty array for other color channels of mask
    empty = np.zeros(processed_output.shape)
    # Stack to make a Green mask where text detected
    mask = np.dstack((empty, processed_output, empty))

    return mask


def create_output_image(image, output):
    '''
    Using input image, and processed output,
    creates an output image showing the result of inference.
    '''
    # Remove final part of output not used for heatmaps
    output = output[:-1]
    # Get only pose detections above 0.5 confidence, set to 255
    for c in range(len(output)):
        output[c] = np.where(output[c]>0.5, 255, 0)
    # Sum along the "class" axis
    output = np.sum(output, axis=0)
    # Get semantic mask
    pose_mask = get_mask(output)
    # Combine with original image
    image = image + pose_mask
    return image
    
def handle_pose_output(output, input_shape):
    '''
    Handles the output of the Pose Estimation model.
    Returns ONLY the keypoint heatmaps, and not the Part Affinity Fields.
    '''
    # TODO 1: Extract only the second blob output (keypoint heatmaps)
    heatmaps = output['Mconv7_stage2_L2']
    # TODO 2: Resize the heatmap back to the size of the input
    # Create an empty array to handle the output map
    out_heatmap = np.zeros([heatmaps.shape[1], input_shape[0], input_shape[1]])
    # Iterate through and re-size each heatmap
    for h in range(len(heatmaps[0])):
        out_heatmap[h] = cv2.resize(heatmaps[0][h], input_shape[0:2][::-1])

    return out_heatmap

def preprocessing(input_image, height, width):
    '''
    Given an input image, height and width:
    - Resize to width and height
    - Transpose the final "channel" dimension to be first
    - Reshape the image to add a "batch" of 1 at the start 
    '''
    image = np.copy(input_image)
    image = cv2.resize(image, (width, height))
    image = image.transpose((2,0,1))
    image = image.reshape(1, 3, height, width)

    return image

def perform_inference(args, i):
    '''
    Performs inference on an input image, given a model.
    '''
    # Create a Network for using the Inference Engine
    inference_network = Network()
    # Load the model in the network, and obtain its input shape
    m = ".\models\human-pose-estimation-0001.xml"
    n, c, h, w = inference_network.load_model(m, args.d, args.c)

    # Get the file name from the path
    input_name = re.split(r'\\', i)[-1].split('.')[-2]

    # Read the input image
    image = cv2.imread(i)

    ### Preprocess the input image
    preprocessed_image = preprocessing(image, h, w)

    # Perform synchronous inference on the image
    inference_network.sync_inference(preprocessed_image)

    # Obtain the output of the inference request
    output = inference_network.extract_output()

    # TODO: Save the raw format of output
    filename = "outputs/{}/rawOutput".format(input_name)
    if not os.path.exists(os.path.dirname(filename)):
        try:
            os.makedirs(os.path.dirname(filename))
        except OSError as exc: # Guard against race condition
            if exc.errno != errno.EEXIST:
                raise

    with open(filename, "wb") as f:
        pickle.dump(output, f)

    ### feeding the output to that function.
    processed_output = handle_pose_output(output, image.shape)

    # Create an output image based on network
    output_image = create_output_image(image, processed_output)

    # Save down the resulting image
    cv2.imwrite("outputs/{}/output.png".format(input_name), output_image)



def main():
    path = ".\images\\"

    images = []
    # r=root, d=directories, f = files
    for r, d, f in os.walk(path):
        for file in f:
            if '.jpg' in file:
                images.append(os.path.join(r, file))

    for i in images:
        args = get_args()
        perform_inference(args, i)


if __name__ == "__main__":
    main()
