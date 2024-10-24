from brisque import BRISQUE
import numpy as np
from PIL import Image
import time
img_path = "2.jpg"
img = Image.open(img_path)
ndarray = np.asarray(img)

obj = BRISQUE(url=False)

a= time.time()
obj.score(img=ndarray)
b= time.time()
print(obj.score(img=ndarray),)