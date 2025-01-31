from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil 
import time
from picamera2 import Picamera2
import cv2
import argparse
import serial.tools.list_ports
from sensors import HCSR04

# Initialize the sensor
front_hcsr04_sensor = HCSR04(trig_pin=23, echo_pin=24)

#------------------------------------------------Pi camera settings---------------------------------------------------------------#
# Desired image dimensions
desired_width = 640
desired_height = 480

# Initialize the camera
picam2 = Picamera2()

# Configure preview settings
picam2.preview_configuration.main.size = (desired_width, desired_height)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.align()

# Configure the camera for preview
picam2.configure("preview")

# Start the camera
picam2.start()

save_path = "Insert Video save path(.avi)"
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter(save_path, fourcc, 60, (desired_width, desired_height))
#---------------------------------------------------------------------------------------------------------------#
# Function to map distance to RC pitch command with smoother transition
def map_distance_to_rc_pitch(distance, threshold=60, pitch_neutral=1500, pitch_min=1000, pitch_max=2000):
    # If the distance is greater than or equal to the threshold, set pitch to neutral (1500)
    if distance >= threshold:
        return pitch_neutral  # No obstacle, pitch stays neutral

    # Calculate a smoother transition using a quadratic or exponential function
    # As the obstacle gets closer, pitch increases but smoothly
    scale_factor = (distance / threshold)  # This gives a value between 0 and 1
    smooth_factor = (1 - scale_factor) ** 1  # Smooth the transition (quadratic)
    
    # Now map this to the pitch range
    pitch_range = pitch_max - pitch_neutral  # Max positive pitch range
    pitch = pitch_neutral + (smooth_factor * pitch_range)  # Gradually increase the pitch as distance decreases

    # Ensure the pitch stays within the valid range
    return int(min(max(pitch, pitch_neutral), pitch_max))

if __name__ == "__main__":
    # Connect to the vehicle
    # Connect to Pixhawk over serial (use the appropriate port and baud rate)
    connection_string = '/dev/ttyAMA0'  # Update if using a different port
    vehicle = connect(connection_string, wait_ready=True, baud=57600)
    print("Connected to Pixhawk")

    
    if vehicle:

            # Main loop
            try:
                while True:

                    # Capture an image from the camera
                    image = picam2.capture_array()
                    if image is not None:
                        image = cv2.flip(image, 0)  # Flip the image if needed
                        image = cv2.flip(image, 1)  # Flip the image if needed

                    # Check if vehicle is in Loiter or Altitude Hold mode
                    mode = vehicle.mode.name
                    #print(f"Current flight mode: {mode}")
                    
                    # Get the distance from the HCSR04 sensor
                    distance = front_hcsr04_sensor.measure()

                    if mode in ["LOITER", "ALT_HOLD"]:
                        #print(f"Obstacle distance: {distance} cm")

                        # If the distance is less than 60 cm, override the RC pitch
                        if distance < 60:
                            # Map the distance to the RC pitch command
                            rc_pitch = map_distance_to_rc_pitch(distance)
                            #print(f"Obstacle detected. Sending RC pitch command: {rc_pitch}")

                            # Override only the pitch channel (channel 2)
                            vehicle.channels.overrides['2'] = rc_pitch  # Channel 2 corresponds to pitch
                        else:
                            # If no obstacle, let the RC pitch control work
                            # No need to override pitch, it will be controlled by the pilot
                            if vehicle.channels.overrides.get('2', None) is not None:
                                vehicle.channels.overrides['2'] = None  # Clear the override to let the pilot control pitch
                            #print("No obstacle detected. RC pitch is active.")

                    # Display distance on OpenCV frame
                
                    cv2.putText(image, f"Front: {distance:.2f} cm", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (45, 25, 2), 6)
                    cv2.putText(image, f"Fly mode: {mode} ", (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (45, 2, 25), 6)
                    cv2.imshow("Distance Measurement", image)

                    out.write(image)

                    # Break the loop on 'q' key press
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break

                    # Sleep for a short period before checking again
                    #time.sleep(0.1)

            except KeyboardInterrupt:
                # Close the vehicle object
                vehicle.close()
                #cleanup
                front_hcsr04_sensor.cleanup()
                cv2.destroyAllWindows()


