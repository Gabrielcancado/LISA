#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from estrutura.srv import seguir, seguirResponse
import cv2
from cv_bridge import CvBridge, CvBridgeError
import mediapipe as mp

mp_hands = mp.solutions.hands
hands = mp_hands.Hands()

def detectar_posicao(imagem):
    try:
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(imagem, "bgr8")
    except CvBridgeError as e:
        rospy.logerr(f"Erro ao converter imagem: {e}")
        return "erro"

    height, width, _ = cv_image.shape

    frame_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
    results = hands.process(frame_rgb)

    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            x_positions = [lm.x for lm in hand_landmarks.landmark]
            if max(x_positions) < 0.5:
                return "esquerda"
            elif min(x_positions) > 0.5:
                return "direita"
    return "centro"

def handle_detect_gesto(req):
    posicao = detectar_posicao(req.imagem)
    return seguirResponse(posicao)

def servidor_gesto():
    rospy.init_node('servidor_gesto')
    s = rospy.Service('detect_gesto', seguir, handle_detect_gesto)
    rospy.loginfo("Servidor de gesto pronto.")
    rospy.spin()

if __name__ == "__main__":
    servidor_gesto()
