import cv2
import numpy as np
import json
import os
import argparse
import sys

def create_hsv_histogram_video(image_dir, output_path, fps=5):
    """
    Generates a video of HSV histograms for polygonal ROIs defined in LabelMe JSON files.

    For each image and its corresponding JSON annotation, this function computes
    the HSV histogram for the area inside the defined polygons and creates a
    video frame showing the image and the histogram side-by-side.

    Args:
        image_dir (str): Path to the directory containing images and JSON files.
        output_path (str): Path to save the output MP4 video.
        fps (int): Frames per second for the output video.
    """
    # Find all JSON files and sort them to ensure a consistent sequence
    try:
        json_files = sorted([f for f in os.listdir(image_dir) if f.endswith('.json')])
    except FileNotFoundError:
        print(f"❌ Error: The directory '{image_dir}' was not found.")
        sys.exit(1)
        
    if not json_files:
        print(f"❌ Error: No JSON files found in '{image_dir}'.")
        return

    video_writer = None
    # Define a fixed frame size: 512x512 for image + 512x512 for histogram
    frame_size = (1024, 512) 
    hist_plot_size = (frame_size[0] // 2, frame_size[1]) # 512x512

    print(f"✅ Found {len(json_files)} JSON files. Starting video creation...")

    for json_filename in json_files:
        base_name = os.path.splitext(json_filename)[0]
        json_path = os.path.join(image_dir, json_filename)

        # Find the corresponding image file (e.g., .jpg, .png)
        image_path = None
        for ext in ['.jpg', '.jpeg', '.png', '.bmp', '.tiff']:
            potential_path = os.path.join(image_dir, base_name + ext)
            if os.path.exists(potential_path):
                image_path = potential_path
                break

        if image_path is None:
            print(f"⚠️ Warning: No corresponding image found for {json_filename}. Skipping.")
            continue

        # 1. Read Image and JSON data
        image = cv2.imread(image_path)
        if image is None:
            print(f"⚠️ Warning: Could not read image {image_path}. Skipping.")
            continue

        with open(json_path, 'r') as f:
            try:
                label_data = json.load(f)
            except json.JSONDecodeError:
                print(f"⚠️ Warning: Could not parse JSON file {json_path}. Skipping.")
                continue

        # 2. Create a combined mask from all polygons in the JSON file
        mask = np.zeros(image.shape[:2], dtype=np.uint8)
        polygons = [np.array(shape['points'], dtype=np.int32) for shape in label_data.get('shapes', [])]

        if not polygons:
            print(f"⚠️ Warning: No shapes found in {json_filename}. Skipping.")
            continue
        
        # Combine all polygons into a single mask for the image
        cv2.fillPoly(mask, polygons, (255))

        # 3. Convert image to HSV and calculate histograms for the masked area
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # cv2.calcHist([image], [channel], mask, [histSize], [ranges])
        hist_h = cv2.calcHist([hsv_image], [0], mask, [180], [0, 180])
        hist_s = cv2.calcHist([hsv_image], [1], mask, [256], [0, 256])
        hist_v = cv2.calcHist([hsv_image], [2], mask, [256], [0, 256])

        # 4. Visualize the histograms on a blank canvas
        hist_h_px, hist_w_px = hist_plot_size[1], hist_plot_size[0]
        hist_img = np.full((hist_h_px, hist_w_px, 3), 255, dtype=np.uint8)
        
        # Normalize histogram values to fit within the plot height
        cv2.normalize(hist_h, hist_h, alpha=0, beta=hist_h_px, norm_type=cv2.NORM_MINMAX)
        cv2.normalize(hist_s, hist_s, alpha=0, beta=hist_h_px, norm_type=cv2.NORM_MINMAX)
        cv2.normalize(hist_v, hist_v, alpha=0, beta=hist_h_px, norm_type=cv2.NORM_MINMAX)

        # Colors for H, S, V histograms (Blue, Green, Red)
        colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]
        hists = [(hist_h, "H"), (hist_s, "S"), (hist_v, "V")]
        
        for i, (hist, label) in enumerate(hists):
            for j, val in enumerate(hist):
                x1 = int((j / len(hist)) * hist_w_px)
                x2 = int(((j + 1) / len(hist)) * hist_w_px)
                y1 = hist_h_px
                y2 = int(hist_h_px - val)
                cv2.rectangle(hist_img, (x1, y1), (x2, y2), colors[i], -1)

        # Add legends and filename to the histogram plot
        cv2.putText(hist_img, "H", (hist_w_px - 80, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, colors[0], 2)
        cv2.putText(hist_img, "S", (hist_w_px - 55, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, colors[1], 2)
        cv2.putText(hist_img, "V", (hist_w_px - 30, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, colors[2], 2)
        cv2.putText(hist_img, os.path.basename(image_path), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,0), 2)

        # 5. Prepare the final frame for the video
        # Draw the polygon outlines on the original image for visualization
        cv2.polylines(image, polygons, isClosed=True, color=(0, 255, 0), thickness=3)
        display_image = cv2.resize(image, (hist_plot_size[0], hist_plot_size[1]))

        # Combine the image and the histogram plot side-by-side
        final_frame = np.hstack((display_image, hist_img))

        # 6. Initialize video writer on the first frame and write to video
        if video_writer is None:
            h, w, _ = final_frame.shape
            fourcc = cv2.VideoWriter_fourcc(*'mp4v') # Codec for .mp4
            video_writer = cv2.VideoWriter(output_path, fourcc, fps, (w, h))
            print(f"🎥 Video properties: {(w,h)} @ {fps} FPS. Saving to {output_path}")

        video_writer.write(final_frame)
        print(f"⚙️  Processed frame for {os.path.basename(image_path)}")

    # 7. Release the video writer resource
    if video_writer:
        video_writer.release()
        print(f"🎉 Video creation complete! Output saved to '{output_path}'")
    else:
        print("No valid frames were processed. Video not created.")


def create_dummy_files(directory):
    """Creates a dummy directory with sample files for demonstration."""
    if os.path.exists(directory):
        return # Do not overwrite existing directory
        
    print(f"Directory '{directory}' not found. Creating dummy files for demonstration.")
    os.makedirs(directory)
    
    # Create dummy image 1 (BGR)
    img1 = np.zeros((400, 600, 3), dtype=np.uint8)
    cv2.rectangle(img1, (100, 50), (300, 250), (255, 100, 100), -1) # Blueish
    cv2.rectangle(img1, (350, 150), (550, 350), (100, 255, 100), -1) # Greenish
    cv2.imwrite(os.path.join(directory, "test_image_1.jpg"), img1)
    
    json_data1 = {"shapes": [{"label": "blue_rect", "points": [[100, 50], [300, 50], [300, 250], [100, 250]]}]}
    with open(os.path.join(directory, "test_image_1.json"), "w") as f:
        json.dump(json_data1, f, indent=2)

    # Create dummy image 2 (BGR)
    img2 = np.zeros((400, 600, 3), dtype=np.uint8)
    cv2.circle(img2, (300, 200), 150, (100, 100, 255), -1) # Reddish
    cv2.imwrite(os.path.join(directory, "test_image_2.png"), img2)
    
    # Approximate a circle with a polygon for the JSON
    points = cv2.ellipse2Poly((300, 200), (150, 150), 0, 0, 360, 20)
    json_data2 = {"shapes": [{"label": "red_circle", "points": points.tolist()}]}
    with open(os.path.join(directory, "test_image_2.json"), "w") as f:
        json.dump(json_data2, f, indent=2)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description="Create a video of HSV histograms for ROIs defined by LabelMe polygons."
    )
    parser.add_argument(
        '-d', '--directory',
        type=str,
        required=True,
        help="Path to the directory containing images and their corresponding LabelMe JSON files."
    )
    parser.add_argument(
        '-o', '--output',
        type=str,
        default='hsv_histogram_sequence.mp4',
        help="Path to save the output MP4 video file. (default: hsv_histogram_sequence.mp4)"
    )
    parser.add_argument(
        '-f', '--fps',
        type=int,
        default=2,
        help="Frames per second for the output video. (default: 2)"
    )

    args = parser.parse_args()

    # If the specified directory doesn't exist, create dummy files for a quick test.
    create_dummy_files(args.directory)

    create_hsv_histogram_video(args.directory, args.output, args.fps)