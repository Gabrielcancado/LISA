#!/usr/bin/python3

import rospy
import cv2
import mediapipe as mp

from estrutura.srv import controle

def count_fingers(image, results):
    # Conta o número de dedos levantados
    finger_count = 0
    for idx in range(5):
        # Polegar
        if results.landmark[4*idx].y > results.landmark[4*idx + 2].y:
            finger_count += 1
    return finger_count

def main():
    rospy.wait_for_service('controle')
    try:
        controle_service = rospy.ServiceProxy('controle', controle)

        mp_hands = mp.solutions.hands
        hands = mp_hands.Hands()
        cap = cv2.VideoCapture(0)

        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break

            # Convertendo a imagem para RGB
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # Detecção das mãos
            results = hands.process(frame_rgb)

            # Contagem de dedos
            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    finger_count = count_fingers(frame, hand_landmarks)
                    if finger_count == 2:
                        # Chama o serviço
                        controle_service(finger_count)

            cv2.imshow('MediaPipe Hands', frame)

            if cv2.waitKey(5) & 0xFF == 27:
                break

        cap.release()
        cv2.destroyAllWindows()

    except rospy.ServiceException as e:
        rospy.logerr(f"Falha ao chamar o serviço: {e}")

if __name__ == "__main__":
    rospy.init_node('cliente_mediapipe')
    main()
