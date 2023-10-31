import tensorflow as tf
import sys

sys.path.append('/home/pawel/Documents/RISA/sem2/ZNiMPRA-Zaawansowane narzedzia i metody programowania robotow autonomicznych/Lab05 Neural Networks/AIProject') 

import ai

train_ds, val_ds = ai.datasets.mnist()

for i in range(100):
    for images, labels in train_ds:
        images = images[..., tf.newaxis]

    for images, labels in val_ds:
        images = images[..., tf.newaxis]
