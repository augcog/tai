# Save this script as, for example, "video_scene_detection.py"

from scenedetect import open_video, SceneManager
from scenedetect.detectors import AdaptiveDetector
from scenedetect.scene_manager import save_images, write_scene_list
import os


def setup_scene_detection(video_path, custom_window_width=50, custom_weights=None):
    video = open_video(video_path)
    scene_manager = SceneManager()

    video_folder = os.path.dirname(video_path)
    images_folder_name = os.path.splitext(os.path.basename(video_path))[0] + "_images"
    images_output_dir = os.path.join(video_folder, images_folder_name)

    os.makedirs(images_output_dir, exist_ok=True)

    if custom_weights is None:
        custom_weights = AdaptiveDetector.Components(delta_hue=0.1, delta_sat=1.0, delta_lum=1.0, delta_edges=1.0)

    adaptive_detector = AdaptiveDetector(
        window_width=custom_window_width,
        weights=custom_weights,
    )

    scene_manager.add_detector(adaptive_detector)

    return video, scene_manager, images_output_dir


# Adjustments are made to the detect_scenes_and_save_images function to return scene times
def detect_scenes_and_save_images(video, scene_manager, images_output_dir):
    scene_manager.detect_scenes(video, show_progress=True)

    scene_list = scene_manager.get_scene_list()
    csv_file_path = os.path.join(images_output_dir, "detected_scenes.csv")
    with open(csv_file_path, 'w', newline='') as csv_file:
        write_scene_list(csv_file, scene_list)

    print(f"Scene list saved to {csv_file_path}")

    image_filenames = save_images(
        scene_list=scene_list,
        video=video,
        num_images=3,
        output_dir=images_output_dir,
    )

    # Prepare the list of start and end times
    scene_times = [(start_time.get_seconds(), end_time.get_seconds()) for start_time, end_time in scene_list]

    for i, ((start_time, end_time), images) in enumerate(zip(scene_list, image_filenames.values()), start=1):
        print(f"Scene {i}: Start Time: {start_time.get_seconds()}, End Time: {end_time.get_seconds()}")
        for image_path in images:
            print(f"  - {image_path}")

    return scene_times


def process_video_scenes(video_path):
    video, scene_manager, images_output_dir = setup_scene_detection(video_path)
    scene_times = detect_scenes_and_save_images(video, scene_manager, images_output_dir)
    return scene_times


# If this script is run directly
if __name__ == "__main__":
    video_path = "Denero/Mutual Recursion/Mutual Recursion.mp4"
    scene_times = process_video_scenes(video_path)
    print("Detected scene times:")
    for start, end in scene_times:
        print(f"Start: {start}, End: {end}")
