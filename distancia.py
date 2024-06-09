import cv2
import numpy as np

# Função para calcular a distância do rosto até a câmera
def calcular_distancia(face_width, focal_length, pixel_width):
    return (face_width * focal_length) / pixel_width

# Matriz de calibração
camera_matrix = np.array([[2.67451856e+18, 0.00000000e+00, 3.57035237e+02],
                          [0.00000000e+00, 2.67451856e+18, 2.94598864e+02],
                          [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

# Coeficientes de distorção
dist_coeffs = np.array([[-1.74548333e-08, 2.10749195e-08, -1.25821264e-09, -4.62049009e-10, 1.88927816e-11]])

# Carregar o classificador de rostos
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

# Inicializar a câmera
cap = cv2.VideoCapture(0)  # Use o índice 2 para a câmera externa

# Largura média do rosto humano em centímetros
average_face_width = 14.0  # cm (aproximadamente)

while True:
    # Capturar um frame
    ret, frame = cap.read()
    if not ret:
        break

    # Converter o frame para escala de cinza
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detectar rostos na imagem
    faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

    for (x, y, w, h) in faces:
        # Verificar se a largura do rosto é maior que um valor mínimo
        if w > 100:
            # Desenhar um retângulo ao redor do rosto
            cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)

            # Calcular a distância do rosto até a câmera
            focal_length = camera_matrix[0, 0]  # Distância focal (fx)
            face_width_in_pixels = w  # Largura do rosto em pixels
            distance = calcular_distancia(average_face_width, focal_length, face_width_in_pixels)

            # Dividir a distância medida por 10^16 para mover o ponto para a direita
            distance /= 10**16
            distance += 19

            # Mostrar a distância sobre o rosto detectado
            cv2.putText(frame, f"Distância: {distance:.2f} cm", (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Mostrar o frame com os rostos detectados
    cv2.imshow('Face Detection', frame)

    # Verificar se a tecla 'q' foi pressionada para sair do loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Liberar os recursos
cap.release()
cv2.destroyAllWindows()
