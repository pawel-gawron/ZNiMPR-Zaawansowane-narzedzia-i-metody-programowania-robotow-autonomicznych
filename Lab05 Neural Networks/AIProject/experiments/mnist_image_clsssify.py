from argparse import ArgumentParser
import tensorflow as tf
import sys

sys.path.append('/home/pawel/Documents/RISA/sem2/ZNiMPRA-Zaawansowane narzedzia i metody programowania robotow autonomicznych/Lab05 Neural Networks/AIProject') 

import ai


def main(args):
    # todo: load mnist dataset
    train_ds, val_ds = ai.datasets.mnist(128)
    train_labels = train_ds.map(lambda x, y: y)
    num_classes = 10

    # todo: create and optimize model (add regularization like dropout and batch normalization)
    model = ai.models.image.ImageClassifier(num_classes)

    # todo: create optimizer (optional: try with learning rate decay)
    optimizer = tf.keras.optimizers.Adam

    # todo: define query function
    def query():
        pass

    # todo: define train function
    def train():
        pass

    # todo: run training and evaluation for number or epochs (from argument parser)
    #  and print results (accumulated) from each epoch (train and val separately)
    ...


if __name__ == '__main__':
    parser = ArgumentParser()
    # todo: pass arguments
    parser.add_argument('--allow-memory-growth', action='store_true', default=False)
    args, _ = parser.parse_known_args()

    if args.allow_memory_growth:
        ai.utils.allow_memory_growth()

    main(args)
