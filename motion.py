import pickle
import matplotlib.pyplot as plt

def load_prediction(path):
    print('Loading ', path)
    with open(path, 'rb') as file:
        out = pickle.load(file)

    heatmaps = out['Mconv7_stage2_L2']
    for heatmap in heatmaps[0]:
        plt.imshow(heatmap)

    heatmap = heatmaps[0][1]
    # plt.imshow(heatmap)
    plt.show()
    print(heatmaps.shape)

import fire
if __name__ == '__main__':
    fire.Fire()
