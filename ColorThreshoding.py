# To add a new cell, type '# %%'
# To add a new markdown cell, type '# %% [markdown]'
# %%
from IPython import get_ipython

null.tpl [markdown]
# # Demo Color Thresholding
# 
# ![](road_img.jpg)

# %%
import cv2
import matplotlib.pyplot as plt
import numpy as np

img = cv2.imread('road_img.jpg')
RGB_im = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
print(RGB_im.shape)
plt.imshow(RGB_im)
plt.show()


# %%
imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)


# %%
mask = cv2.inRange(imgHSV, (20, 80, 70), (50, 255, 255))
result = cv2.bitwise_and(img, img, mask=mask)
plt.imshow(result)
plt.show()
cv2.imwrite('green_th.png', result)
cv2.imshow('H', result)
cv2.waitKey(0)
cv2.destroyAllWindows()


# %%
get_ipython().run_line_magic('pinfo', 'cv2.inRange')


# %%



