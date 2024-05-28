#!/usr/bin/env python3

import rospy
import cv2
import mediapipe as mp
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from estrutura.srv import seguir

mp_hands = mp.solutions.hands
hands = mp_hands.Hands()
bridge = CvBridge()

def count_fingers(results):
    finger_count = 0
    for hand_landmarks in results.multi_hand_landmarks:
        if hand_landmarks.landmark[4].y < hand_landmarks.landmark[3].y:  # Polegar
            finger_count += 1
        if hand_landmarks.landmark[8].y < hand_landmarks.landmark[6].y:  # Indicador
            finger_count += 1
        if hand_landmarks.landmark[12].y < hand_landmarks.landmark[10].y:  # Médio
            finger_count += 1
        if hand_landmarks.landmark[16].y < hand_landmarks.landmark[14].y:  # Anelar
            finger_count += 1
        if hand_landmarks.landmark[20].y < hand_landmarks.landmark[18].y:  # Mínimo
            finger_count += 1
    return finger_count

def main():
    rospy.init_node('cliente_gesto')
    rospy.wait_for_service('detect_gesto')

    try:
        detect_gesto = rospy.ServiceProxy('detect_gesto', seguir)

        cap = cv2.VideoCapture(0)

        while not rospy.is_shutdown() and cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break

            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = hands.process(frame_rgb)

            if results.multi_hand_landmarks:
                finger_count = count_fingers(results)
                if finger_count == 3:
                    try:
                        ros_image = bridge.cv2_to_imgmsg(frame, "bgr8")
                        response = detect_gesto(ros_image)
                        rospy.loginfo(f"Posição do usuário: {response.posicao}")
                    except rospy.ServiceException as e:
                        rospy.logerr(f"Falha ao chamar o serviço: {e}")
                    

            cv2.imshow('MediaPipe Hands', frame)
            if cv2.waitKey(5) & 0xFF == 27:
                break

        cap.release()
        cv2.destroyAllWindows()

    except rospy.ServiceException as e:
        rospy.logerr(f"Falha ao conectar ao serviço: {e}")

if __name__ == "__main__":
    main()
