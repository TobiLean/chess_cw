import cv2
import numpy as np
from roboflow import Roboflow
import supervision as sv
from ultralytics import YOLO
import matplotlib.pyplot as plt

# Initialize the Roboflow object for piece detection
rf = Roboflow(api_key="U8ywvD0Vh3VIHQ3eCRx8")
project = rf.workspace().project("yolo-chess-dvcho")
model_pieces = project.version(7).model

# Initialize the YOLO model for board detection
model_board = YOLO('ChessBoardModel.pt')


def detect_board(image_path):
    # Run inference on the source image for board detection
    results = model_board(source=image_path, conf=0.7)

    # Check if any objects were detected
    if results and len(results[0].boxes.xyxy) > 0:
        # Get the bounding box coordinates of the first detected chessboard
        x1, y1, x2, y2 = results[0].boxes.xyxy[0]

        # Define width and height of each cell
        cell_width = (x2 - x1) / 8
        cell_height = (y2 - y1) / 8

        # Create a list of cell coordinates
        cells = []
        for j in range(8):
            row = []
            for i in range(8):
                cell_x1 = int(x1 + i * cell_width)
                cell_y1 = int(y1 + j * cell_height)
                cell_x2 = int(cell_x1 + cell_width)
                cell_y2 = int(cell_y1 + cell_height)
                row.append(((cell_x1, cell_y1), (cell_x2, cell_y2)))
            cells.append(row)
        return cells
    return None


def detect_pieces(image_path):
    # Perform inference for piece detection
    result = model_pieces.predict(image_path, confidence=40, overlap=30).json()
    predictions = result['predictions']
    return predictions


def map_pieces_to_positions(image_path, cells, predictions):
    image = cv2.imread(image_path)
    files = 'ABCDEFGH'
    pieces_positions = []

    # Draw grid on every square
    for j, row in enumerate(cells):
        for i, cell in enumerate(row):
            cell_x1, cell_y1 = cell[0]
            cell_x2, cell_y2 = cell[1]
            cv2.rectangle(image, (cell_x1, cell_y1), (cell_x2, cell_y2), (0, 255, 0), 2)
            cell_label = f"{files[i]}{8 - j}"
            cv2.putText(image, cell_label, (cell_x1 + 5, cell_y1 + 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)

    for prediction in predictions:
        x, y = prediction['x'], prediction['y']
        class_name = prediction['class']

        for j, row in enumerate(cells):
            for i, cell in enumerate(row):
                cell_x1, cell_y1 = cell[0]
                cell_x2, cell_y2 = cell[1]
                if cell_x1 < x < cell_x2 and cell_y1 < y < cell_y2:
                    position = f"{files[i]}{8 - j}"
                    pieces_positions.append((class_name, position))

                    # Add piece label on the image
                    cv2.putText(image, f"{class_name} {position}", (cell_x1 + 5, cell_y1 + 50),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)

    # Show the annotated image with Matplotlib
    plt.figure(figsize=(16, 16))
    plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
    plt.title("Detected Pieces and Positions")
    plt.show()

    return pieces_positions

# Main function to integrate everything
def main():
    image_path = 'top_1.jpg'
    cells = detect_board(image_path)
    if cells:
        predictions = detect_pieces(image_path)
        pieces_positions = map_pieces_to_positions(image_path, cells, predictions)
        print("Detected pieces and their positions:", pieces_positions)
    else:
        print("Chessboard not detected or no bounding boxes found.")


# Run the main function
if __name__ == "__main__":
    main()

