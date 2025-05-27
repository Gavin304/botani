import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import cv2
import openai
import os
import json
from dotenv import load_dotenv
import base64
from openai import OpenAI

class ros2yolo(Node):
    def __init__(self):
        super().__init__('ros2yolo')
        self.publisher = self.create_publisher(Point, 'botani/coordinates', 10)

        # Load .env from absolute workspace root
        env_path = '/home/gavin/botani/.env'
        load_dotenv(env_path)

        self.api_key = os.getenv('OPENAI_API_KEY')
        if not self.api_key:
            raise ValueError("OPENAI_API_KEY not found")

        self.client = OpenAI(api_key=self.api_key)
        self.capture_and_process_image()

    def capture_and_process_image(self):
        # Open the camera
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            self.get_logger().error("Unable to access the camera")
            return

        # Capture a single frame
        ret, frame = cap.read()
        cap.release()

        if not ret:
            self.get_logger().error("Failed to capture image")
            return

        # Save the image temporarily
        image_path = '/tmp/captured_image.jpg'
        cv2.imwrite(image_path, frame)

        # Load the prompt template
        prompt_path = os.path.join(os.path.dirname(__file__), 'prompt.txt')
        try:
            with open(prompt_path, 'r') as prompt_file:
                prompt_template = prompt_file.read()
        except FileNotFoundError:
            self.get_logger().error(f"Prompt file not found: {prompt_path}")
            return

        # Send the image and prompt to ChatGPT Vision (gpt-4.1-mini)
        try:
            prompt = prompt_template.format(image_description="an image of plants")
            with open(image_path, "rb") as image_file:
                image_bytes = image_file.read()
            image_b64 = base64.b64encode(image_bytes).decode("utf-8")
            image_url = "data:image/jpeg;base64," + image_b64

            response = self.client.responses.create(
                model="gpt-4.1-mini",
                input=[{
                    "role": "user",
                    "content": [
                        {"type": "input_text", "text": prompt},
                        {
                            "type": "input_image",
                            "image_url": image_url,
                        },
                    ],
                }],
            )
            data = response.output_text
            self.get_logger().info(f"OpenAI raw response: {data}")
            if not data:
                self.get_logger().error("OpenAI API returned empty response.")
                return
        except Exception as e:
            self.get_logger().error(f"Failed to send image to OpenAI API: {e}")
            return

        # Parse the JSON response for plant names and coordinates
        try:
            plants = json.loads(data)  # Ensure the response is valid JSON
            for plant in plants:
                name = plant.get('name', 'unknown')
                x = plant.get('x', 0.0)
                y = plant.get('y', 0.0)

                # Publish the coordinates
                point = Point()
                point.x = x
                point.y = y
                point.z = 0.0  # Set z to 0.0 as it's not required
                self.publisher.publish(point)
                self.get_logger().info(f"Published plant: {name}, x={x}, y={y}")
        except Exception as e:
            self.get_logger().error(f"Error parsing API response: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ros2yolo()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()