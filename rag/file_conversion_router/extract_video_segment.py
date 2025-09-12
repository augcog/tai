#!/usr/bin/env python3
import subprocess
import sys
import os

def extract_video_segment(input_video, start_time, end_time, output_video=None):
    """
    Extract a segment from a video file using ffmpeg.
    
    Args:
        input_video: Path to the input video file
        start_time: Start time in format "HH:MM:SS" or seconds
        end_time: End time in format "HH:MM:SS" or seconds
        output_video: Path to the output video file (optional)
    """
    if not os.path.exists(input_video):
        print(f"Error: Input video file '{input_video}' not found")
        return False
    
    # Generate output filename if not provided
    if output_video is None:
        base_name = os.path.splitext(os.path.basename(input_video))[0]
        ext = os.path.splitext(input_video)[1]
        output_video = f"{base_name}_segment{ext}"
    
    # Build ffmpeg command
    cmd = [
        'ffmpeg',
        '-i', input_video,
        '-ss', str(start_time),
        '-to', str(end_time),
        '-c', 'copy',  # Copy codec (no re-encoding for speed)
        '-avoid_negative_ts', 'make_zero',
        output_video,
        '-y'  # Overwrite output file if it exists
    ]
    
    print(f"Extracting segment from {start_time} to {end_time}...")
    print(f"Input: {input_video}")
    print(f"Output: {output_video}")
    
    try:
        result = subprocess.run(cmd, capture_output=True, text=True)
        if result.returncode == 0:
            print(f"Success! Segment saved to: {output_video}")
            return True
        else:
            print(f"Error: {result.stderr}")
            return False
    except FileNotFoundError:
        print("Error: ffmpeg not found. Please install ffmpeg first.")
        print("On Ubuntu/Debian: sudo apt-get install ffmpeg")
        print("On macOS: brew install ffmpeg")
        return False
    except Exception as e:
        print(f"Error: {e}")
        return False

if __name__ == "__main__":
    # Your specific video
    input_video = "/home/bot/bot/yk/YK_final/courses/CS 294-137/2020 CS 294-137-20241018T023838Z-009/2020 CS 294-137/Videos - Published to Class/Vision/Vision 4_1.mp4"
    start_time = "0:00"  # 4 seconds
    end_time = "1:22"    # 1 minute 15 seconds
    
    # Extract the segment
    extract_video_segment(input_video, start_time, end_time)