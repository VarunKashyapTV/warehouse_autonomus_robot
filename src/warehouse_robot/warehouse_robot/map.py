from PIL import Image
from matplotlib import pyplot as plt
import os

os.path.expanduser("~/map.pgm")
data = Image.open(os.path.expanduser("~/map.pgm"))
plt.imshow(data)
plt.show()