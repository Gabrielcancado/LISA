#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Bool
import subprocess
import os
import re

current_process = None
current_gif = 'animated_standard.gif'
last_state = None

def play_gif(gif_name):
    global current_process, current_gif
    try:
        # Define o caminho absoluto para o diretório das imagens
        gif_path = os.path.join('/home/gabriel/lisa_ws/src/estrutura/Images/telas/', gif_name)

        # Verifica se o arquivo existe
        if not os.path.exists(gif_path):
            rospy.logerr(f"O arquivo GIF não foi encontrado: {gif_path}")
            return

        # Certifique-se de que a variável DISPLAY está configurada para :0
        os.environ['DISPLAY'] = ':0'

        # Comando para executar o mpv com um método de saída de vídeo específico
        command = ['mpv', '--fullscreen', '--loop=inf', '--vo=x11', gif_path]

        # Termina o processo anterior, se houver
        if current_process:
            current_process.terminate()
            current_process.wait()  # Espera o término completo do processo

        # Executa o comando
        current_process = subprocess.Popen(command)
        current_gif = gif_name
    except Exception as e:
        rospy.logerr(f"Erro ao tentar exibir o GIF: {e}")

def estado_callback(data):
    global last_state
    if data.data != last_state:
        last_state = data.data
        if data.data:
            rospy.loginfo("Estado mudou para True. Exibindo animated_standard_cursor.gif.")
            play_gif('animated_standard_cursor.gif')
        else:
            rospy.loginfo("Estado mudou para False. Exibindo animated_standard.gif.")
            play_gif('animated_standard.gif')

def resultados_callback(data):
    gesture_mapping = {
        'Thumb_Up': 'grooving',
        'Thumb_Down': 'angry',
        'Open_Palm': 'tropelo',
        'Closed_Fist': 'sleepy',
        'Victory': 'inlove',
        'Pointing_Up': 'victory',
    }

    match = re.search(r'Gesto reconhecido: Gesto (\w+) reconhecido 5 vezes seguidas', data.data)
    if match:
        gesture_name = match.group(1)
        if gesture_name in gesture_mapping:
            rospy.loginfo(f"Gesto reconhecido: {gesture_name}. Exibindo animated_{gesture_mapping[gesture_name]}.gif.")
            play_gif(f'animated_{gesture_mapping[gesture_name]}.gif')
        else:
            rospy.logwarn(f"Gesto '{gesture_name}' não encontrado no mapeamento.")
    else:
        rospy.logwarn("Formato da mensagem não reconhecido.")

def gif_display_node():
    rospy.init_node('gif_display_node', anonymous=True)
    rospy.Subscriber('/estado', Bool, estado_callback)
    rospy.Subscriber('/resultados', String, resultados_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        gif_display_node()
    except rospy.ROSInterruptException:
        pass
