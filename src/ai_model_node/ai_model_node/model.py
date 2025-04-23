import torch
import cv2
from pathlib import Path
import os

def rel_path(filepath):

    return Path(os.path.dirname(__file__), filepath)


def test_read(fp):
    with open(rel_path(fp), "r") as fs:
        x = fs.readline()

    return x




class PytorchModel():
    def __init__(self, model_class, weights_path, model_config={}):
        self.model_class = model_class
        self.weights_path = weights_path
        self.device = 'cuda' if torch.cuda.is_available() else 'mps' if torch.backends.mps.is_available() else 'cpu'
        self.model = None 

        if model_config:
            self.model_configuration = model_config
    
    def load_model(self):
        
        self.model = self.model_class(**self.model_configuration)
        self.model.load_stata_dict(torch.load(self.weights_path, map_location=self.device))
        self.model.to(self.device).eval()

        return self.device, 1 # return 1 for successful run, and the device it's running on. 