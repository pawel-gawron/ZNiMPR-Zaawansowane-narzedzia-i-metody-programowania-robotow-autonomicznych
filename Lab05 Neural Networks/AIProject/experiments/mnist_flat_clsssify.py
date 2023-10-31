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
    print(train_labels)

    # todo: create and optimize model (add regularization like dropout and batch normalization)
    model = ai.models.image.FlatImageClassifier(num_classes)

    # todo: create optimizer (optional: try with learning rate decay)
    initial_learning_rate = 0.0001
    lr_schedule = tf.keras.optimizers.schedules.ExponentialDecay(
        initial_learning_rate,
        decay_steps=10000,
        decay_rate=0.96,
        staircase=True)
    optimizer = tf.keras.optimizers.Adam(learning_rate=lr_schedule)

    # todo: define query function
    def query(images, labels, training):
        predictions = model(images, training)
        loss = ai.losses.classification_loss(labels, predictions)

        return loss

    # todo: define train function
    def train(images, labels):
        with tf.GradientTape() as tape:
            loss = query(images, labels, True)
            gradients = tape.gradient(loss, model.trainable_variables)
            optimizer.apply_gradients(zip(gradients, model.trainable_variables))
        return loss

    # todo: run training and evaluation for number or epochs (from argument parser)
    #  and print results (accumulated) from each epoch (train and val separately)

    for i in range(100):
        for images, labels in train_ds:
            train(images, labels)


if __name__ == '__main__':
    parser = ArgumentParser()
    # todo: pass arguments
    parser.add_argument('--allow-memory-growth', action='store_true', default=False)
    args, _ = parser.parse_known_args()

    if args.allow_memory_growth:
        ai.utils.allow_memory_growth()

    main(args)
