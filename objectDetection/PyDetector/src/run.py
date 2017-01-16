import os
from config import *

# Link to the UIUC Car Database
# http://l2r.cs.uiuc.edu/~cogcomp/Data/Car/CarData.tar.gz
dataset_url = "http://l2r.cs.uiuc.edu/~cogcomp/Data/Car/CarData.tar.gz"
dataset_path = "../data/dataset/CarData.tar.gz"

if not os.path.exists(os.path.split(dataset_path)[0]):
    os.makedirs(os.path.split(dataset_path)[0])

# Fetch and extract the dataset
if not os.path.exists(dataset_path):
    os.system("wget {} -O {}".format(dataset_url, dataset_path))
    os.system("tar -xvzf {} -C {}".format(dataset_path, os.path.split(dataset_path)[0]))

# Extract the features
pos_path = "../data/dataset/CarData/TrainImages"
neg_path = "../data/dataset/CarData/TrainImages"
os.system("python ../src/extract-features.py -p {} -n {}".format(pos_path, neg_path))

# Perform training
os.system("python ../src/train-classifier.py -p {} -n {}".format(pos_feat_ph, neg_feat_ph))

# Perform testing
test_im_path = "../data/dataset/CarData/TestImages/test-16.pgm"
os.system("python ../src/test-classifier.py -i {} --visualize".format(test_im_path))
