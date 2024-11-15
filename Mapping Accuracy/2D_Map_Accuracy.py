import cv2
import matplotlib.pyplot as plt
import numpy as np

def resize_image_to_match(img1, img2):
    height, width = img2.shape[:2]
    resized_img1 = cv2.resize(img1, (width, height))
    return resized_img1

def translate_image(img, x_translation, y_translation):
    rows, cols = img.shape[:2]
    M = np.float32([[1, 0, x_translation], [0, 1, y_translation]])
    translated_img = cv2.warpAffine(img, M, (cols, rows))
    return translated_img

def zoom_image(img, scale_factor):
    rows, cols = img.shape[:2]
    M = np.float32([[scale_factor, 0, 0], [0, scale_factor, 0]])
    zoomed_img = cv2.warpAffine(img, M, (int(cols * scale_factor), int(rows * scale_factor)))
    new_rows, new_cols = zoomed_img.shape[:2]
    center_x, center_y = new_cols // 2, new_rows // 2
    crop_x1, crop_y1 = max(center_x - cols // 2, 0), max(center_y - rows // 2, 0)
    crop_x2, crop_y2 = crop_x1 + cols, crop_y1 + rows
    zoomed_cropped_img = zoomed_img[crop_y1:crop_y2, crop_x1:crop_x2]
    return zoomed_cropped_img

def zoom_out_image(img, scale_factor):
    rows, cols = img.shape[:2]
    M = np.float32([[scale_factor, 0, 0], [0, scale_factor, 0]])
    zoomed_img = cv2.warpAffine(img, M, (int(cols * scale_factor), int(rows * scale_factor)))
    zoomed_img_resized = cv2.resize(zoomed_img, (cols, rows))
    return zoomed_img_resized

def calculate_metrics(ground_truth, map_image):
    ground_truth_binary = (ground_truth > 0).astype(np.uint8)
    map_binary = (map_image > 0).astype(np.uint8)
    
    TP = np.sum((ground_truth_binary == 1) & (map_binary == 1))
    TN = np.sum((ground_truth_binary == 0) & (map_binary == 0))
    FP = np.sum((ground_truth_binary == 0) & (map_binary == 1))
    FN = np.sum((ground_truth_binary == 1) & (map_binary == 0))

    metrics = {
        'TP': (ground_truth_binary == 1) & (map_binary == 1),
        'TN': (ground_truth_binary == 0) & (map_binary == 0),
        'FP': (ground_truth_binary == 0) & (map_binary == 1),
        'FN': (ground_truth_binary == 1) & (map_binary == 0)
    }

    return metrics, TP, TN, FP, FN

def overlap_images(image_path1, image_path2):
    img1 = cv2.imread(image_path1)
    img2 = cv2.imread(image_path2)
    
    if img1 is None or img2 is None:
        print("Error: Could not open or find one of the images.")
        return None
    
    img1_zoomed_out = zoom_out_image(img1, 0.8)
    img1_resized = resize_image_to_match(img1_zoomed_out, img2)
    
    img2_zoomed = zoom_image(img2, 1.28)
    img2_translated = translate_image(img2_zoomed, -14.75,10)
    
    gray1 = cv2.cvtColor(img1_resized, cv2.COLOR_BGR2GRAY)
    gray2 = cv2.cvtColor(img2_translated, cv2.COLOR_BGR2GRAY)
    
    overlapped = cv2.addWeighted(gray1, 0.5, gray2, 0.5, 0)
    
    plt.imshow(overlapped, cmap='gray')
    plt.title('Overlapped Grayscale Image')
    plt.axis('on')  # Show axis
    plt.show()
    
    # Calculate metrics
    metrics, TP, TN, FP, FN = calculate_metrics(gray1, gray2)
    
    print(f"True Positives (TP): {TP}")
    print(f"True Negatives (TN): {TN}")
    print(f"False Positives (FP): {FP}")
    print(f"False Negatives (FN): {FN}")
    print(f"Map Accuracy: {100*(TP + TN) / (TP + TN + FP + FN):.2f}")
    # Display the metrics
    for key, value in metrics.items():
        cv2.imshow(f"{key} Pixels", value.astype(np.uint8) * 255)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    image_path1 = '/home/nithish8055/Pictures/maps_pngs/gazebo_env_final.png'  # Update this path
    image_path2 = '/home/nithish8055/Pictures/maps_pngs/final_maps/thinned_frontier10_test_mapped_image.png'  # Update this path
    overlap_images(image_path1, image_path2)
    

