import cv2
import numpy as np

def detectar_bordes_linea_roja(img_bgr):
    """
    Detecta los bordes de la línea roja usando máscara HSV + Canny.
    Devuelve la imagen de bordes (binaria).
    """
    # 1) Convertir BGR → HSV y enmascarar rojo
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    lower1 = np.array([0, 100, 50]);   upper1 = np.array([10, 255, 255])
    lower2 = np.array([170, 100, 50]); upper2 = np.array([180, 255, 255])
    m1 = cv2.inRange(hsv, lower1, upper1)
    m2 = cv2.inRange(hsv, lower2, upper2)
    mask_red = cv2.bitwise_or(m1, m2)

    # 2) Extraer solo la zona roja
    rojo_extraido = cv2.bitwise_and(img_bgr, img_bgr, mask=mask_red)

    # 3) Pasar a gris y suavizar
    gray = cv2.cvtColor(rojo_extraido, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)

    # 4) Canny
    edges = cv2.Canny(blur, threshold1=50, threshold2=150, apertureSize=3)
    return edges

def main():
    ruta = 'track.png'
    img = cv2.imread(ruta)
    if img is None:
        print(f"No se pudo cargar la imagen {ruta}")
        return

    # Detectar bordes en la línea roja
    edges = detectar_bordes_linea_roja(img)

    # Vista original + overlay de bordes en verde
    overlay = img.copy()
    overlay[edges != 0] = (0, 255, 0)

    # Vista solo track sobre fondo negro
    track_negro = np.zeros_like(img)          # imagen negra
    track_negro[edges != 0] = (255, 255, 255)  # línea en blanco

    # Mostrar todas las ventanas
    cv2.imshow('Original', img)
    cv2.imshow('Bordes Canny (línea roja)', edges)
    cv2.imshow('Overlay de bordes en verde', overlay)
    cv2.imshow('Track sobre fondo negro', track_negro)

    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
