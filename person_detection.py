import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO
import time

class PersonDetector:
    def __init__(self):
        """Initialize the person detector with YOLOv8 and RealSense camera"""
        # Initialize YOLOv8 model
        try:
            self.model = YOLO('yolov8n.pt')  # Load the smallest YOLOv8 model
            print("YOLOv8 model loaded successfully")
        except Exception as e:
            print(f"Error loading YOLOv8 model: {e}")
            print("Please install ultralytics: pip install ultralytics")
            raise
        
        # Initialize RealSense pipeline
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        # Configure streams
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        
        # Start pipeline
        try:
            self.profile = self.pipeline.start(self.config)
            print("RealSense camera initialized successfully")
        except Exception as e:
            print(f"Error initializing RealSense camera: {e}")
            raise
        
        # Get depth sensor for depth scaling
        depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()
        
        # Create align object to align depth frames to color frames
        align_to = rs.stream.color
        self.align = rs.align(align_to)
        
        print("Person detection system ready!")
    
    def detect_persons(self, frame):
        """Detect persons in the given frame using YOLOv8"""
        results = self.model(frame, verbose=False)
        
        persons_detected = []
        for result in results:
            boxes = result.boxes
            if boxes is not None:
                for box in boxes:
                    # Get class and confidence
                    cls = int(box.cls[0])
                    conf = float(box.conf[0])
                    
                    # Class 0 is 'person' in COCO dataset
                    if cls == 0 and conf > 0.5:  # Confidence threshold of 0.5
                        # Get bounding box coordinates
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        persons_detected.append({
                            'bbox': (int(x1), int(y1), int(x2), int(y2)),
                            'confidence': conf
                        })
        
        return persons_detected
    
    def get_depth_at_point(self, depth_frame, x, y):
        """Get depth value at a specific point"""
        try:
            depth = depth_frame.get_distance(x, y)
            return depth
        except:
            return None
    
    def run_detection(self):
        """Main detection loop"""
        print("Starting person detection... Press 'q' to quit")
        
        try:
            while True:
                # Wait for a new frame
                frames = self.pipeline.wait_for_frames()
                
                # Align depth frame to color frame
                aligned_frames = self.align.process(frames)
                
                # Get aligned frames
                color_frame = aligned_frames.get_color_frame()
                depth_frame = aligned_frames.get_depth_frame()
                
                if not color_frame or not depth_frame:
                    continue
                
                # Convert to numpy arrays
                color_image = np.asanyarray(color_frame.get_data())
                depth_image = np.asanyarray(depth_frame.get_data())
                
                # Detect persons
                persons = self.detect_persons(color_image)
                
                # Process detections
                if persons:
                    print("Person detected")
                    
                    # Draw bounding boxes and get depth information
                    for person in persons:
                        x1, y1, x2, y2 = person['bbox']
                        conf = person['confidence']
                        
                        # Draw bounding box
                        cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        
                        # Add confidence text
                        cv2.putText(color_image, f'Person: {conf:.2f}', 
                                  (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 
                                  0.5, (0, 255, 0), 2)
                        
                        # Get depth at center of bounding box
                        center_x = (x1 + x2) // 2
                        center_y = (y1 + y2) // 2
                        depth = self.get_depth_at_point(depth_frame, center_x, center_y)
                        
                        if depth:
                            # Add depth information
                            cv2.putText(color_image, f'Depth: {depth:.2f}m', 
                                      (x1, y2+20), cv2.FONT_HERSHEY_SIMPLEX, 
                                      0.5, (255, 255, 0), 2)
                
                # Display the image
                cv2.imshow('Person Detection', color_image)
                
                # Break loop on 'q' press
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                
                # Small delay to prevent excessive CPU usage
                time.sleep(0.01)
                
        except KeyboardInterrupt:
            print("\nDetection stopped by user")
        except Exception as e:
            print(f"Error during detection: {e}")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up resources"""
        self.pipeline.stop()
        cv2.destroyAllWindows()
        print("Person detection system stopped")

def main():
    """Main function to run person detection"""
    try:
        detector = PersonDetector()
        detector.run_detection()
    except Exception as e:
        print(f"Failed to start person detection: {e}")
        print("Make sure your RealSense camera is connected and YOLOv8 is installed")

if __name__ == "__main__":
    main()
