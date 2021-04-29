import numpy as np
from PIL import Image as im


env_map = np.ones((300, 300))*255




# 60
# 84


# 45
env_map[10:(300-30),55:65]=0
env_map[10:(300-30),80:90]=0
# Image.fromarray(b)
env_img = im.fromarray((env_map).astype(np.uint8))

env_img.save('env_obstacles.png')