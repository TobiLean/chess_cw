import numpy as np
import time
import logging
from Arm_Lib import Arm_Device

class DofbotChessController:
    def __init__(self, board_dimensions=(174, 174)):
        # CRITICAL: Set default attributes BEFORE any method calls
        self.Z_HOVER_HEIGHT = 20  # DEFAULT hover height - FIRST attribute set
        self.GRIP_OPEN = 90
        self.GRIP_CLOSE = 180
        self.MOVEMENT_SPEED = 1000

        # Board dimension calculations
        self.BOARD_WIDTH, self.BOARD_LENGTH = board_dimensions
        self.SQUARE_WIDTH = self.BOARD_WIDTH / 8
        self.SQUARE_HEIGHT = self.BOARD_LENGTH / 8

        # Logging configuration
        logging.basicConfig(
            level=logging.INFO, 
            format='%(asctime)s - Chess Movement - %(levelname)s: %(message)s'
        )
        
        # Arm connection
        self.arm = Arm_Device()
        
        # Generate coordinates AFTER all attributes are set
        self.board_coordinates = self._generate_board_coordinates()
        
        # Debug print of board coordinates
        print("Generated Board Coordinates:")
        for square, coords in self.board_coordinates.items():
            print(f"{square}: {coords}")
    
    def turn_on_torque(self, enable):
        if enable == 0:
            self.arm.Arm_serial_set_torque(0)
            print("Torque off")
        elif enable == 1:
            self.arm.Arm_serial_set_torque(1)
            print("Torque on")

    def _generate_board_coordinates(self):
        """
        Simplified coordinate mapping for chess squares
        """
        coordinates = {}
        files = 'ABCDEFGH'
        ranks = range(1, 9)
        
        # Predefined origin servo angles
        origin_angles = {
            1: 53,   # Base Rotation
            2: 117,  # Shoulder
            3: -10,  # Elbow
            4: 9,    # Wrist Horizontal
            5: 89,   # Wrist Rotation
            6: 122   # Gripper
        }
        
        for file_index, file in enumerate(files):
            for rank_index, rank in enumerate(ranks):
                square_name = f"{file}{rank}"
                
                # Simple coordinate calculation
                coordinates[square_name] = {
                    'servo_angles': {
                        1: origin_angles[1] + (7 - file_index) * 5,  # Adjust base rotation
                        2: origin_angles[2],
                        3: origin_angles[3],
                        4: origin_angles[4],
                        5: origin_angles[5],
                        6: origin_angles[6]
                    },
                    'board_coords': (
                        (7 - file_index) * (self.BOARD_WIDTH / 8),
                        rank_index * (self.BOARD_LENGTH / 8),
                        0
                    )
                }
        
        return coordinates

    def test_h1_to_g1_movement(self):
        """
        Comprehensive movement and diagnostic test
        """
        try:
            # Explicitly print out h1 and g1 details
            print("H1 Details:")
            h1_details = self.board_coordinates.get('h1'.upper())
            if h1_details is None:
                raise ValueError("No coordinates found for h1")
            
            print("H1 Servo Angles:", h1_details.get('servo_angles', 'No angles found'))
            print("H1 Board Coordinates:", h1_details.get('board_coords', 'No coordinates found'))
            
            print("\nG1 Details:")
            g1_details = self.board_coordinates.get('g1'.upper())
            if g1_details is None:
                raise ValueError("No coordinates found for g1")
            
            print("G1 Servo Angles:", g1_details.get('servo_angles', 'No angles found'))
            print("G1 Board Coordinates:", g1_details.get('board_coords', 'No coordinates found'))
            
            # Move to g1
            print("\nMoving to G1...")
            g1_angles = g1_details['servo_angles']
            for servo, angle in g1_angles.items():
                print(f"Moving Servo {servo} to {angle} degrees")
                self.arm.Arm_serial_servo_write(servo, angle, self.MOVEMENT_SPEED)
            
            time.sleep(2)  # Allow movement to complete
            
            # Log position after moving to g1
            print("\nPosition after moving to G1:")
            self.log_current_arm_position()
            
            # Move back to h1
            print("\nMoving back to H1...")
            h1_angles = h1_details['servo_angles']
            for servo, angle in h1_angles.items():
                print(f"Moving Servo {servo} to {angle} degrees")
                self.arm.Arm_serial_servo_write(servo, angle, self.MOVEMENT_SPEED)
            
            time.sleep(2)  # Allow movement to complete
            
            # Final position check
            print("\nFinal Position (back to H1):")
            self.log_current_arm_position()
        
        except Exception as e:
            print(f"Movement test failed: {e}")
            import traceback
            traceback.print_exc()
            
    def test_h2_movement(self):
        """
        Comprehensive movement test to H2 square
        """
        try:
            # Explicitly print out h1 and h2 details
            print("H1 Details:")
            h1_details = self.board_coordinates.get('H1')
            if h1_details is None:
                raise ValueError("No coordinates found for h1")
            
            print("H1 Servo Angles:", h1_details.get('servo_angles', 'No angles found'))
            print("H1 Board Coordinates:", h1_details.get('board_coords', 'No coordinates found'))
            
            print("\nH2 Details:")
            h2_details = self.board_coordinates.get('H2')
            if h2_details is None:
                raise ValueError("No coordinates found for h2")
            
            print("H2 Servo Angles:", h2_details.get('servo_angles', 'No angles found'))
            print("H2 Board Coordinates:", h2_details.get('board_coords', 'No coordinates found'))
            
            # Move to h2
            print("\nMoving to H2...")
            h2_angles = h2_details['servo_angles']
            for servo, angle in h2_angles.items():
                print(f"Moving Servo {servo} to {angle} degrees")
                self.arm.Arm_serial_servo_write(servo, angle, self.MOVEMENT_SPEED)
            
            time.sleep(2)  # Allow movement to complete
            
            # Log position after moving to h2
            print("\nPosition after moving to H2:")
            self.log_current_arm_position()
            
            # Optional: Move back to home or h1
            print("\nMoving back to initial position...")
            # Move back to h1
            print("\nMoving back to H1...")
            h1_angles = h1_details['servo_angles']
            for servo, angle in h1_angles.items():
                print(f"Moving Servo {servo} to {angle} degrees")
                self.arm.Arm_serial_servo_write(servo, angle, self.MOVEMENT_SPEED)
            
            time.sleep(2)  # Allow movement to complete
            
            # Final position check
            print("\nFinal Position (back to H1):")
            self.log_current_arm_position()
            del self.arm
            
        except Exception as e:
            print(f"Movement test failed: {e}")
            import traceback
            traceback.print_exc()

    def _calculate_square_angles(self, file_index, rank_index):
        """
        Calculate servo angles for a specific square
        Interpolate from origin point
        """
        # Create a copy of origin angles to modify
        square_angles = self.ORIGIN_SERVO_ANGLES.copy()
        
        # Adjust base rotation for horizontal movement
        # Move left as we go from h to a (decreasing file index)
        square_angles[1] = self.ORIGIN_SERVO_ANGLES[1] + (7 - file_index) * 5
        
        # Adjust shoulder and elbow for vertical movement
        square_angles[2] = self.ORIGIN_SERVO_ANGLES[2] - rank_index * 2
        square_angles[3] = self.ORIGIN_SERVO_ANGLES[3] - rank_index * 2
        
        return square_angles

    def precise_inverse_kinematics(self, x, y, z):
        # Segment lengths
        L1 = 9.5  # Servo 2 to 3
        L2 = 9.5  # Servo 3 to 4
        L3 = 9.5  # Servo 4 to 5
        L4 = 11.0 # Gripper length

        # Total reach calculation
        total_length = L1 + L2 + L3 + L4

        # Explicit home position handling
        if np.isclose(x, 0) and np.isclose(y, 0) and np.isclose(z, 0):
            return [
                90,  # Servo 1: Base Rotation - explicitly set to 90
                90,  # Servo 2: Shoulder Joint - explicitly set to 90
                90,  # Servo 3: Elbow Joint - explicitly set to 90
                90,  # Servo 4: Wrist Horizontal - explicitly set to 90
                90,  # Servo 5: Wrist Rotation - explicitly set to 90
                self.GRIP_OPEN  # Servo 6: Gripper
            ]

        # Calculate base rotation (horizontal plane)
        base_angle = np.degrees(np.arctan2(y, x))
        base_angle = max(0, min(180, base_angle))

        # Calculate radial distance from base
        radial_distance = np.sqrt(x**2 + y**2)
        total_distance = np.sqrt(radial_distance**2 + z**2)

        # Ensure the point is within arm's reach
        if total_distance > total_length:
            raise ValueError(f"Point {(x,y,z)} is outside arm's reach of {total_length} cm")

        # Law of Cosines to calculate joint angles
        # This is a simplified approach and might need fine-tuning
        try:
            # Angle between ground and arm
            theta = np.degrees(np.arctan2(z, radial_distance))

            # Shoulder and elbow angle calculations
            cos_gamma = (L1**2 + L2**2 - (total_distance - L1)**2) / (2 * L1 * L2)
            gamma = np.degrees(np.arccos(cos_gamma))

            # Shoulder angle (relative to ground)
            shoulder_angle = theta + np.degrees(np.arccos(
                (L1**2 + total_distance**2 - (L2 + L3 + L4)**2) / (2 * L1 * total_distance)
            ))

            # Elbow angle
            elbow_angle = 180 - gamma

            joint_angles = [
                base_angle,        # Servo 1: Base Rotation
                90 - shoulder_angle,  # Servo 2: Shoulder Joint
                elbow_angle,       # Servo 3: Elbow Joint
                90,                # Servo 4: Wrist Horizontal
                90,                # Servo 5: Wrist Rotation
                self.GRIP_OPEN     # Servo 6: Gripper
            ]

            # Ensure angles are within servo limits
            joint_angles = [max(0, min(180, angle)) for angle in joint_angles[:4] + [joint_angles[5]] +
                            [max(0, min(270, angle)) for angle in [joint_angles[4]]]]

            return joint_angles

        except Exception as e:
            logging.error(f"Inverse Kinematics Calculation Error: {e}")
            raise ValueError(f"Unable to calculate movement angles for coordinates: {x}, {y}, {z}")

    def test_movement_precision(self):
        # Test Points: Verify movement to various positions
        test_points = [
            (0, 0, 0),     # Theoretical home position
            (4, 0, 0),     # Straight forward
            (0, 4, 0),     # Straight to the side
            (0, 0, 4),     # Straight up
            (4, 4, 4)      # Diagonal movement
        ]
        
        for point in test_points:
            print(f"\nTesting Movement to Point: {point}")
            
            try:
                # Calculate Required Joint Angles
                joint_angles = self.precise_inverse_kinematics(*point)
                print("Calculated Joint Angles:", joint_angles)
                
                # Perform the Movement
                self._move_to_coordinates(point)
                
                # Optional: Add user confirmation
                input("Observe movement. Press Enter to continue...")
            
            except Exception as e:
                print(f"Movement Test Error for point {point}: {e}")
    
    def log_current_arm_position(self):
        """
        Comprehensively log the current arm position
        """
        try:
            # Read current servo angles
            current_angles = [
                self.arm.Arm_serial_servo_read(servo) 
                for servo in range(1, 7)
            ]
            
            print("\n--- Current Arm Position ---")
            servo_names = [
                "Base Rotation", 
                "Shoulder", 
                "Elbow", 
                "Wrist Horizontal", 
                "Wrist Rotation", 
                "Gripper"
            ]
            
            # Print each servo's angle with its name
            for i, (name, angle) in enumerate(zip(servo_names, current_angles), 1):
                print(f"Servo {i} ({name}): {angle} degrees")
                
        except Exception as e:
            print(f"Error reading arm position: {e}")

    def set_custom_zero_position(self, x, y, z):
        """
        Set a custom zero/home position above the chess board
        
        Args:
            x (float): X coordinate of new zero point
            y (float): Y coordinate of new zero point
            z (float): Z coordinate of new zero point
        """
        try:
            # Calculate joint angles for this new zero point
            joint_angles = self.precise_inverse_kinematics(x, y, z)
            
            # Move to this position
            for servo_index, angle in enumerate(joint_angles, 1):
                self.arm.Arm_serial_servo_write(
                    servo_index, 
                    angle, 
                    self.MOVEMENT_SPEED
                )
            
            time.sleep(1)  # Allow movement to complete
            
            logging.info(f"Set new zero position to coordinates: ({x}, {y}, {z})")
            
        except Exception as e:
            logging.error(f"Failed to set custom zero position: {e}")

    def move_piece(self, start_square, end_square):
        """
        Complete chess piece movement sequence with safety checks
        
        Args:
            start_square (str): Starting square (e.g., 'E2')
            end_square (str): Destination square (e.g., 'E4')
        """
        try:
            logging.info(f"Moving piece from {start_square} to {end_square}")
            
            # Retrieve precise coordinates
            start_coords = self.board_coordinates[start_square]['xyz']
            end_coords = self.board_coordinates[end_square]['xyz']
            
            # Hover before movement
            self._move_to_coordinates(
                self.board_coordinates[start_square]['hover_xyz']
            )
            
            # Lower to piece
            self._move_to_coordinates(start_coords)
            
            # Grip piece
            self.arm.Arm_serial_servo_write(6, self.GRIP_CLOSE, self.MOVEMENT_SPEED)
            time.sleep(0.5)
            
            # Lift piece
            self._move_to_coordinates(
                self.board_coordinates[start_square]['hover_xyz']
            )
            
            # Move to destination hover
            self._move_to_coordinates(
                self.board_coordinates[end_square]['hover_xyz']
            )
            
            # Lower to destination
            self._move_to_coordinates(end_coords)
            
            # Release piece
            self.arm.Arm_serial_servo_write(6, self.GRIP_OPEN, self.MOVEMENT_SPEED)
            time.sleep(0.5)
            
            logging.info("Piece movement completed successfully")
        
        except Exception as e:
            logging.error(f"Piece movement failed: {e}")
            self._emergency_stop()

    def _move_to_coordinates(self, coordinates):
        """
        Move arm to specific 3D coordinates
        
        Args:
            coordinates (tuple): x, y, z target position
        """
        x, y, z = coordinates
        joint_angles = self.precise_inverse_kinematics(x, y, z)
        
        for servo_index, angle in enumerate(joint_angles, 1):
            self.arm.Arm_serial_servo_write(
                servo_index, 
                angle, 
                self.MOVEMENT_SPEED
            )
        
        time.sleep(0.5)  # Allow movement completion
        
    def set_home_position(self):
        """
        Explicitly set the robot to its true home position
        
        Home position: All joints set to 90 degrees (neutral/default state)
        """
        # Systematic home position setting
        for servo in range(1, 7):  # All 6 servos
            # Use slow movement speed to ensure precision
            self.arm.Arm_serial_servo_write(servo, 90, 500)  # Slower speed for accuracy
        
        # Add a deliberate pause to ensure position is reached
        time.sleep(1)
        
        logging.info("Robot moved to home/neutral position")
        
    def diagnose_home_position(self):
        """
        Comprehensive diagnostic method to investigate home position setting
        """
        print("Diagnostic Home Position Check")
        print("-" * 40)
        
        # Check individual servo movements
        for servo in range(1, 7):
            try:
                print(f"\nTesting Servo {servo}")
                
                # Move servo to 90 degrees
                print(f"Moving Servo {servo} to 90 degrees")
                self.arm.Arm_serial_servo_write(servo, 90, 1000)
                
                # Add a short pause to observe
                time.sleep(0.5)
                
                # Optional: Add some logging or user confirmation
                input(f"Observe Servo {servo}. Does it move as expected? Press Enter to continue...")
            
            except Exception as e:
                print(f"Error with Servo {servo}: {e}")
        
        print("\nDiagnostic Complete")

    def verify_arm_device(self):
        """
        Verify the Arm_Device connection and basic functionality
        """
        try:
            # Check if arm device is responsive
            print("Checking Arm Device Connection")
            
            # Attempt a simple movement
            test_servo = 1
            print(f"Testing Servo {test_servo}")
            
            # Try multiple approaches
            print("Approach 1: Direct Servo Write")
            self.arm.Arm_serial_servo_write(test_servo, 90, 1000)
            
            # Add additional connection verification if possible
            print("Arm Device appears to be responsive")
        
        except Exception as e:
            print(f"Error with Arm Device: {e}")
            print("Possible connection or initialization issue detected")

    def test_arm_movements(self):
        """
        Systematically test arm movements to verify inverse kinematics
        """
        # First, explicitly set home position
        self.set_home_position()
        
        # Test points to verify arm behavior
        test_points = [
            (0, 0, 0),     # Should be close to home position
            (4, 0, 0),     # Straight forward
            (0, 4, 0),     # Straight to the side
            (0, 0, 4),     # Straight up
            (4, 4, 4),     # Diagonal movement
        ]
        
        for point in test_points:
            try:
                print(f"\nTesting point: {point}")
                angles = self.precise_inverse_kinematics(*point)
                print("Calculated Angles:", angles)
                
                # Actually move to the point
                self._move_to_coordinates(point)
                time.sleep(1)  # Pause to observe movement
            
            except Exception as e:
                print(f"Error testing point {point}: {e}")
        
    def calibrate_servo_home_positions(self):
        """
        Systematically identify the true home positions for each servo
        
        Goal: Find the precise angles that represent the neutral position for servos 2 and 5
        """
        # Test range of angles for servos 2 and 5
        test_servos = [2, 5]
        angle_range = range(0, 181, 15)  # From 0 to 180 in 15-degree increments
        
        for servo in test_servos:
            print(f"\nCalibrating Servo {servo}")
            for angle in angle_range:
                print(f"Testing angle {angle}")
                
                # Move entire arm to home position first
                for s in range(1, 7):
                    if s == servo:
                        self.arm.Arm_serial_servo_write(s, angle, self.MOVEMENT_SPEED)
                    else:
                        self.arm.Arm_serial_servo_write(s, 90, self.MOVEMENT_SPEED)
                
                # Pause to observe
                time.sleep(1)
                
                # Prompt for user observation
                input(f"Observe Servo {servo} at angle {angle}. Press Enter to continue...")

    def set_custom_home_position(self):
        """
        Set a custom home position that accounts for servos 2 and 5's unique requirements
        
        Based on calibration results, we'll use specific angles for servos 2 and 5
        """
        # Placeholder values - you'll determine the exact angles during calibration
        custom_home_angles = {
            1: 90,  # Base rotation
            2: 90,  # Shoulder - might need adjustment
            3: 90,  # Elbow
            4: 90,  # Wrist horizontal
            5: 90,  # Wrist rotation - might need adjustment
            6: 90   # Gripper
        }
        
        # Move to custom home position
        for servo, angle in custom_home_angles.items():
            self.arm.Arm_serial_servo_write(servo, angle, self.MOVEMENT_SPEED)
        
        time.sleep(1)  # Ensure position is reached
        logging.info("Robot moved to custom home position")
    
            
def test_initialization():
    """
    Diagnostic function to test controller initialization
    """
    logging.basicConfig(level=logging.DEBUG)
    
    try:
        controller = DofbotChessController()
        print("Initialization Successful!")
        
        # Verify board coordinates generation
        for square, coords in list(controller.board_coordinates.items())[:3]:
            print(f"{square} Coordinates: {coords}")
    
    except Exception as e:
        print(f"Initialization Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    # Create your chess controller instance
    controller = DofbotChessController()

    # Option 1: Quick Device Verification controller.verify_arm_device()

    # Option 2: Detailed Home Position Diagnosis controller.diagnose_home_position()

    # Option 3: Comprehensive Servo Calibration controller.calibrate_servo_home_positions()
    
    # controller.test_movement_precision()
    
    # controller.turn_on_torque(1)
    
    # controller.test_h1_to_g1_movement()
    
    # controller.log_current_arm_position()
()
    
    
