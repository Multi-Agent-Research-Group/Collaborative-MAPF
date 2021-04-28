import numpy as np
from PIL import Image as im


env_map = np.ones((300, 300))

env_map[0,0]=0

env_img = im.fromarray(env_map)

env_img.save('env.png')