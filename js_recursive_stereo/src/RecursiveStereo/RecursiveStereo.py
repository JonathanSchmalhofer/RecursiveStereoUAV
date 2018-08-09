import numpy as np
import cv2

class RecursiveStereo:
    def __init__(self):
        self.left_image  = None
        self.right_image = None
    
    def SetLeftImage(self, image):
        self.left_image = image
    
    def SetRightImage(self, image):
        self.right_image = image
    
    def RequirementsFulfilled(self):
        if self.left_image is None:
            return False
        if self.right_image is None:
            return False
        return True
    
    def Step(self):
        print("TEST")
        if self.RequirementsFulfilled():
            print("Everything ok")
        else:
            print("Something is missing")