#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import cv2
import mediapipe as mp
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

# Função para calcular a distância do rosto até a câmera
def calcular_distancia(face_width, focal_length, pixel_width):
    return (face_width * focal_length) / pixel_width

# Parâmetros de calibração da câmera
camera_matrix = np.array([[6.73013110e+02, 0.00000000e+00, 1.74499999e+02],
                          [0.00000000e+00, 6.72565563e+02, 1.49500000e+02],
                          [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

# Largura média do rosto humano em centímetros
average_face_width = 14.0  # cm (aproximadamente)

class FaceDirectionNode:
    def __init__(self):
        rospy.loginfo("Initializing FaceDirectionNode")
        self.bridge = CvBridge()
        rospy.loginfo("CvBridge initialized")
        self.face_detection = mp.solutions.face_detection.FaceDetection(min_detection_confidence=0.2)
        rospy.loginfo("FaceDetection initialized")
        self.image_sub = rospy.Subscriber("/Imagens", Image, self.image_callback)
        rospy.loginfo("Subscriber initialized")
        self.center_x_pub = rospy.Publisher("/face_center_x", Float32, queue_size=10)
        self.center_y_pub = rospy.Publisher("/face_center_y", Float32, queue_size=10)
        self.distance_pub = rospy.Publisher('/face_distance', Float32, queue_size=10)
        rospy.loginfo("Publishers initialized")

    def image_callback(self, msg):
        rospy.loginfo("Received image message")
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "passthrough")
            rospy.loginfo("Image converted to OpenCV format")
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
            return

        # Converte a imagem para RGB
        image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        rospy.loginfo("Image converted to RGB")

        # Processa a imagem para detectar rostos
        results = self.face_detection.process(image_rgb)
        rospy.loginfo("Image processed for face detection")

        if results.detections:
            rospy.loginfo(f"Detected {len(results.detections)} faces")
            for detection in results.detections:
                # Obtém as coordenadas do centro do rosto
                bbox = detection.location_data.relative_bounding_box
                height, width, _ = frame.shape
                x_center = int(bbox.xmin * width + (bbox.width * width) / 2)
                y_center = int(bbox.ymin * height + (bbox.height * height) / 2)

                # Calcula a largura do rosto em pixels
                face_width_in_pixels = int(bbox.width * width)

                # Calcula a distância do rosto até a câmera
                focal_length = camera_matrix[0, 0]  # Distância focal (fx)
                distance = calcular_distancia(average_face_width, focal_length, face_width_in_pixels)

                # Publica a distância do rosto
                self.distance_pub.publish(distance)

                # Se a distância for menor que 1 metro, calcula e publica as coordenadas
                if distance <= 100:
                    # Calcula as distâncias do centro do rosto até o centro da imagem
                    rect_center_x = width // 2
                    rect_center_y = height // 2
                    distance_x = x_center - rect_center_x
                    distance_y = y_center - rect_center_y

                    # Publica as coordenadas
                    self.center_x_pub.publish(Float32(distance_x))
                    self.center_y_pub.publish(Float32(distance_y))
                    rospy.loginfo(f"Published center_x={distance_x}, center_y={distance_y}")
                else:
                    rospy.loginfo("Face is farther than 1 meter, ignoring coordinates.")
        else:
            rospy.loginfo("No face detected")

def main():
    rospy.init_node('face_direction_node')
    face_direction_node = FaceDirectionNode()
    rospy.loginfo("FaceDirectionNode node initialized, spinning...")
    rospy.spin()

if __name__ == '__main__':
    main()
