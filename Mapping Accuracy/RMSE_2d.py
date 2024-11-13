from PIL import Image
import numpy as np
from sklearn.metrics import mean_squared_error

# Load the images
image1_path = '/home/nithish8055/Pictures/maps_pngs/final_maps/thinned_hector2_test_mapped_image.png'
image2_path = '/home/nithish8055/Pictures/maps_pngs/final_maps/thinned_frontier2_test_mapped_image.png'
image1 = Image.open(image1_path).convert('L')
image2 = Image.open(image2_path).convert('L')

# Convert images to numpy arrays
image1_array = np.array(image1)
image2_array = np.array(image2)

# Resize images to be the same shape if they are not already
if image1_array.shape != image2_array.shape:
    image2_array = np.resize(image2_array, image1_array.shape)

# Calculate RMSE
mse = mean_squared_error(image1_array.flatten(), image2_array.flatten())
rmse = np.sqrt(mse)

print(f"RMSE: {rmse}")

