import numpy as np
import time
import logging
import keyboard
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

        # Predefined precise coordinates for known squares
        self.PRECISE_COORDINATES = {
            'h1': (54, 140, -30, 30, 270, 176),
            'g1': (62, 140, -30, 25, 270, 176),
            'f1': (72, 140, -30, 25, 270, 176),
            'e1': (82, 140, -30, 25, 270, 176),
            'd1': (94, 140, -30, 25, 270, 176),
            'c1': (106, 140, -30, 25, 270, 176),
            'b1': (116, 140, -30, 25, 270, 176),
            'a1': (124, 140, -30, 30, 270, 176),

            # Second Row
            'h2': (60, 120, -20, 35, 270, 176),
            'g2': (68, 120, -25, 40, 270, 176),
            'f2': (76, 120, -30, 45, 270, 176),
            'e2': (84, 120, -35, 50, 270, 176),
            'd2': (92, 120, -40, 55, 270, 176),
            'c2': (100, 120, -45, 60, 270, 176),
            'b2': (108, 120, -50, 65, 270, 176),
            'a2': (120, 120, -20, 35, 270, 176),

            # Third Row
            'h3': (62, 120, -40, 65, 270, 176),
            'g3': (70, 120, -45, 70, 270, 176),
            'f3': (78, 120, -50, 75, 270, 176),
            'e3': (86, 120, -55, 80, 270, 176),
            'd3': (94, 120, -60, 85, 270, 176),
            'c3': (102, 120, -65, 90, 270, 176),
            'b3': (110, 120, -70, 95, 270, 176),
            'a3': (116, 120, -40, 65, 270, 176),

            # Fourth Row
            'h4': (64, 100, -35, 90, 270, 176),
            'g4': (72, 100, -40, 95, 270, 176),
            'f4': (80, 100, -45, 100, 270, 176),
            'e4': (88, 100, -50, 105, 270, 176),
            'd4': (96, 100, -55, 110, 270, 176),
            'c4': (104, 100, -60, 115, 270, 176),
            'b4': (112, 100, -65, 120, 270, 176),
            'a4': (112, 100, -35, 90, 270, 176),

            # Fifth Row
            'h5': (66, 90, -30, 95, 270, 176),
            'g5': (74, 90, -35, 100, 270, 176),
            'f5': (82, 90, -40, 105, 270, 176),
            'e5': (90, 90, -45, 110, 270, 176),
            'd5': (98, 90, -50, 115, 270, 176),
            'c5': (106, 90, -55, 120, 270, 176),
            'b5': (114, 90, -60, 125, 270, 176),
            'a5': (110, 90, -30, 95, 270, 176),

            # Sixth Row
            'h6': (68, 75, -25, 110, 270, 176),
            'g6': (76, 75, -30, 115, 270, 176),
            'f6': (84, 75, -35, 120, 270, 176),
            'e6': (92, 75, -40, 125, 270, 176),
            'd6': (100, 75, -45, 130, 270, 176),
            'c6': (108, 75, -50, 135, 270, 176),
            'b6': (116, 75, -55, 140, 270, 176),
            'a6': (108, 75, -25, 110, 270, 176),

            # Seventh Row
            'h7': (70, 60, -20, 125, 270, 176),
            'g7': (78, 60, -25, 130, 270, 176),
            'f7': (86, 60, -30, 135, 270, 176),
            'e7': (94, 60, -35, 140, 270, 176),
            'd7': (102, 60, -40, 145, 270, 176),
            'c7': (110, 60, -45, 150, 270, 176),
            'b7': (118, 60, -50, 155, 270, 176),
            'a7': (106, 60, -20, 125, 270, 176),

            # Eighth Row (Bottom Row)
            'h8': (72, 50, -15, 135, 270, 176),
            'g8': (78, 50, -15, 135, 270, 176),
            'f8': (82, 50, -15, 135, 270, 176),
            'e8': (88, 50, -15, 135, 270, 176),
            'd8': (92, 50, -15, 135, 270, 176),
            'c8': (96, 50, -15, 135, 270, 176),
            'b8': (102, 50, -15, 135, 270, 176),
            'a8': (106, 50, -15, 135, 270, 176)
            
        }

        # Logging configuration
        logging.basicConfig(
            level=logging.INFO, 
            format='%(asctime)s - Chess Movement - %(levelname)s: %(message)s'
        )
        
        # Arm connection
        self.arm = Arm_Device()
        
        # Generate coordinates by filling in the gaps
        #self.board_coordinates = self._generate_board_coordinates()
        
        # Debug print of board coordinates
        print("Board Coordinates:")
        for square, coords in self.PRECISE_COORDINATES.items():
            print(f"{square}: {coords}")

    def _generate_board_coordinates(self):
        """
        Generate board coordinates by interpolating between known points
        and using precise pre-defined coordinates where available
        """
        coordinates = {}
        files = 'ABCDEFGH'
        ranks = range(1, 9)
        
        for file_index, file in enumerate(files):
            for rank_index, rank in enumerate(ranks):
                square_name = f"{file}{rank}"
                
                # Check if we have a precise coordinate for this square
                if square_name in self.PRECISE_COORDINATES:
                    precise_angles = self.PRECISE_COORDINATES[square_name]
                    coordinates[square_name] = {
                        'servo_angles': {
                            1: precise_angles[0],
                            2: precise_angles[1],
                            3: precise_angles[2],
                            4: precise_angles[3],
                            5: precise_angles[4],
                            6: precise_angles[5]
                        },
                        'board_coords': (
                            (7 - file_index) * (self.BOARD_WIDTH / 8),
                            rank_index * (self.BOARD_LENGTH / 8),
                            0
                        )
                    }
                else:
                    # Interpolation for missing squares using a simple algorithm
                    # This is a placeholder and should be refined with actual measurements
                    base_angle = 53 + (7 - file_index) * 5
                    shoulder_base = 117
                    elbow_base = -10
                    
                    coordinates[square_name] = {
                        'servo_angles': {
                            1: base_angle,
                            2: shoulder_base + (-rank_index * 2),
                            3: elbow_base + (-rank_index * 2),
                            4: 9,  # Placeholder - needs precise measurement
                            5: 89,  # Placeholder - needs precise measurement
                            6: 122  # Gripper position
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
        Comprehensive movement and diagnostic test from H1 to G1
        
        This method demonstrates:
        1. Retrieving precise coordinates for H1 and G1
        2. Performing a movement test between these squares
        3. Providing detailed logging and error handling
        """
        try:
            # Validate and log details for H1
            print("H1 Details:")
            h1_details = self.board_coordinates.get('H1')
            if h1_details is None:
                raise ValueError("No coordinates found for H1")
            
            h1_servo_angles = h1_details.get('servo_angles', {})
            h1_board_coords = h1_details.get('board_coords', 'No coordinates found')
            
            print("H1 Servo Angles:", h1_servo_angles)
            print("H1 Board Coordinates:", h1_board_coords)
            
            # Validate and log details for G1
            print("\nG1 Details:")
            g1_details = self.board_coordinates.get('G1')
            if g1_details is None:
                raise ValueError("No coordinates found for G1")
            
            g1_servo_angles = g1_details.get('servo_angles', {})
            g1_board_coords = g1_details.get('board_coords', 'No coordinates found')
            
            print("G1 Servo Angles:", g1_servo_angles)
            print("G1 Board Coordinates:", g1_board_coords)
            
            # Validate servo angles
            required_servos = [1, 2, 3, 4, 5, 6]
            for servo in required_servos:
                if servo not in h1_servo_angles or servo not in g1_servo_angles:
                    raise ValueError(f"Missing servo angle for servo {servo}")
            
            # Move to G1
            print("\nMoving to G1...")
            for servo, angle in g1_servo_angles.items():
                print(f"Moving Servo {servo} to {angle} degrees")
                self.arm.Arm_serial_servo_write(servo, angle, self.MOVEMENT_SPEED)
            
            time.sleep(2)  # Allow movement to complete
            
            # Move back to H1
            print("\nMoving back to H1...")
            for servo, angle in h1_servo_angles.items():
                print(f"Moving Servo {servo} to {angle} degrees")
                self.arm.Arm_serial_servo_write(servo, angle, self.MOVEMENT_SPEED)
            
            time.sleep(2)  # Allow movement to complete
            
            print("\nMovement test completed successfully.")
        
        except Exception as e:
            print(f"Movement test failed: {e}")
            import traceback
            traceback.print_exc()
            
    def move_to_cell(self, cell_id):
        """
        Move the robot arm to a specific chess board cell using Arm_serial_servo_write6.
        
        This method provides a flexible way to navigate to any cell on the chessboard
        with optimized servo movement.
        
        Parameters:
        -----------
        cell_id : str
            The chess board cell identifier (e.g., 'A1', 'H8', 'E4')
            Must be a valid chess notation cell name
        
        Behavior:
        ---------
        - If the cell exists in PRECISE_COORDINATES, moves to that cell
        - If the cell does not exist, prints an error message
        - Validates cell notation before attempting movement
        """
        # Normalize cell ID to uppercase for consistent lookup
        cell_id = cell_id.lower()
        
        # Basic validation of chess notation
        if len(cell_id) != 2:
            print(f"Error: Invalid cell notation '{cell_id}'. Must be two characters (e.g., 'A1', 'H8').")
            return
        
        # Check if first character is a valid file (A-H)
        if cell_id[0] not in 'abcdefgh':
            print(f"Error: Invalid file '{cell_id[0]}'. Must be a-h.")
            return
        
        # Check if second character is a valid rank (1-8)
        if cell_id[1] not in '12345678':
            print(f"Error: Invalid rank '{cell_id[1]}'. Must be 1-8.")
            return
        
        # Check if cell exists in precise coordinates
        if cell_id not in self.PRECISE_COORDINATES:
            print(f"Error: No precise coordinates found for cell '{cell_id}'.")
            print("Available cells in precise coordinates:")
            for available_cell in sorted(self.PRECISE_COORDINATES.keys()):
                print(available_cell, end=', ')
            print("\n")
            return
        
        # Retrieve precise coordinates for the cell
        cell_coordinates = self.PRECISE_COORDINATES[cell_id]
        
        try:
            # Prepare servo angles for Arm_serial_servo_write6
            # The method expects angles for all 6 servos in order
            servo_angles = list(cell_coordinates)
            
            print(f"\nMoving to cell {cell_id}...")
            print(f"Servo Angles: {servo_angles}")
            
            # Use Arm_serial_servo_write6 for simultaneous servo movement
            # Parameters: servo1, servo2, servo3, servo4, servo5, servo6, speed
            self.arm.Arm_serial_servo_write6(
                servo_angles[0],  # Servo 1 (Base Rotation)
                servo_angles[1],  # Servo 2 (Shoulder)
                servo_angles[2],  # Servo 3 (Elbow)
                servo_angles[3],  # Servo 4 (Wrist Horizontal)
                servo_angles[4],  # Servo 5 (Wrist Rotation)
                servo_angles[5],  # Servo 6 (Gripper)
                self.MOVEMENT_SPEED  # Movement speed
            )
            
            time.sleep(2)  # Allow movement to complete
            
            print(f"Successfully moved to cell {cell_id}.")
        
        except Exception as e:
            print(f"Error during movement to {cell_id}: {e}")
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

    def iterate_chessboard(self):
        files = 'HGFEDCBA'
        ranks = range(1, 9)

        # Iterate through each rank
        for rank in ranks:
            # Determine the order for files based on the rank
            file_order = files if rank % 2 == 1 else files[::-1]

            # Iterate through each file in the determined order
            for file in file_order:
                cell = f"{file}{rank}"
                print(f"Current cell: {cell}")
                self.move_to_cell(cell)
                input("Press space to continue...")  # Wait for space key press to move to the next cell

            
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
    
    #controller.move_to_cell('h1')
    
    # Iterate through chessboard
    controller.iterate_chessboard()
    
    # controller.test_h1_to_g1_movement()
    
    # controller.log_current_arm_position()
    
