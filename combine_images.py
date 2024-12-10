import cv2 as cv
import numpy as np
from pathlib import Path
import os

thispath = Path(__file__).resolve()

scene = "scene_meeting_2"

datadir = Path(thispath.parent / "output_video")

output_path = Path(datadir / "combined" / scene)
Path(output_path).mkdir(parents=True, exist_ok=True)

images_scene_dir = sorted(Path(datadir / scene).rglob('*.png'))
images_scene_segmented_dir = sorted(Path(datadir / "mask3d" / scene).rglob('*.png'))

for image_scene_path, image_scene_segmented_path in zip(images_scene_dir, images_scene_segmented_dir):
    # because path is object not string
    image_scene = cv.imread(str(image_scene_path))
    image_scene_segmented = cv.imread(str(image_scene_segmented_path))
    concatenated_image = np.concatenate((image_scene, image_scene_segmented), axis=0)
    cv.imwrite(f"{output_path}/{image_scene_path.name}", concatenated_image)

# # Use FFmpeg to convert images to video
os.system(f"ffmpeg -framerate 30 -i {output_path}/frame_%03d.png -c:v libx264 -pix_fmt yuv420p {output_path}/scene_video.mp4")