import numpy as np
import matplotlib.pyplot as plt
from skimage.morphology import disk, dilation

'''
Authors:

Rahul Sajnani
Amarthya Sasi Kiran
Abhiram Kadiyala

Date: 5th February 2021
'''




def plot_images(images_list, plot_dim, cmap="viridis", title = None, subplot_names = None):
    '''
    Function to plot images
    '''
    assert len(images_list) <= (plot_dim[0] * plot_dim[1]), "Number of images is more than the plot dimensions"
    
    fig = plt.figure(figsize=(10,7), dpi= 100, facecolor='w', edgecolor='k')
    plt.subplots_adjust(left=None, bottom=None, right=None, top=None, wspace=None, hspace=0.5)
    
    if title is not None:
        fig.suptitle(title, fontsize=15)
    
    index = 1
        
    for image in images_list:
        
        plt.subplot(plot_dim[0], plot_dim[1], index)
        if subplot_names is not None:
            plt.gca().set_title(subplot_names[index - 1])
        plt.imshow(image, cmap = cmap)
        index += 1
        
    plt.show()