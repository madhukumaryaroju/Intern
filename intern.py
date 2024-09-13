import cv2
import os
import datetime
import numpy as np

def apply_filter(frame, filter_type):
    if filter_type == 'grayscale':
        return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    elif filter_type == 'sepia':
        sepia_filter = cv2.transform(frame, np.matrix([[0.272, 0.534, 0.131], 
                                                       [0.349, 0.686, 0.168], 
                                                       [0.393, 0.769, 0.189]]))
        return np.clip(sepia_filter, 0, 255).astype(np.uint8)
    else:
        return frame

def add_timestamp(frame):
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    cv2.putText(frame, timestamp, (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    return frame

def capture_images_from_webcam(output_folder, num_images=5, video_duration=10):
    # Create output directory if it doesn't exist
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    # Start capturing video from webcam
    cap = cv2.VideoCapture(0)  # 0 is typically the default camera

    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return

    print("Press 'q' to quit, 's' to save an image, 'v' to start/stop video recording, and 'f' to apply filter.")
    
    img_count = 0
    recording = False
    video_writer = None
    filter_type = None

    while img_count < num_images:
        # Capture frame-by-frame
        ret, frame = cap.read()

        # Check if frame is captured
        if not ret:
            print("Error: Failed to capture image.")
            break

        # Apply filter if any
        if filter_type:
            frame = apply_filter(frame, filter_type)
        
        # Add timestamp to the frame
        frame = add_timestamp(frame)

        # Display the resulting frame
        cv2.imshow('Webcam', frame)

        # Wait for user input
        key = cv2.waitKey(1)
        if key == ord('q'):  # Press 'q' to quit
            break
        elif key == ord('s'):  # Press 's' to save image
            img_filename = os.path.join(output_folder, f'image_{img_count + 1}.jpg')
            cv2.imwrite(img_filename, frame)
            print(f'Image saved as {img_filename}')
            img_count += 1
        elif key == ord('v'):  # Press 'v' to start/stop video recording
            if recording:
                recording = False
                video_writer.release()
                print('Video recording stopped.')
            else:
                recording = True
                video_filename = os.path.join(output_folder, f'video_{datetime.datetime.now().strftime("%Y%m%d_%H%M%S")}.avi')
                fourcc = cv2.VideoWriter_fourcc(*'XVID')
                video_writer = cv2.VideoWriter(video_filename, fourcc, 20.0, (frame.shape[1], frame.shape[0]))
                print(f'Video recording started: {video_filename}')
        elif key == ord('f'):  # Press 'f' to apply filter
            if filter_type == 'grayscale':
                filter_type = 'sepia'
                print('Filter: Sepia')
            elif filter_type == 'sepia':
                filter_type = None
                print('Filter: None')
            else:
                filter_type = 'grayscale'
                print('Filter: Grayscale')
        
        # Write the frame to video if recording
        if recording:
            video_writer.write(frame)

    # Release the capture and close windows
    if recording:
        video_writer.release()
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    # Prompt the user to enter the output directory
    output_folder = input("Enter the directory where you want to save the images and videos: ")
    
    # Capture images and record video from the webcam
    capture_images_from_webcam(output_folder)
