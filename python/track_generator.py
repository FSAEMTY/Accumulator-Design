import cv2
import numpy as np
from skimage.morphology import skeletonize

def detectar_bordes_linea_roja(img_bgr):
    """
    Detecta los bordes de la línea roja usando máscara HSV + Canny.
    Devuelve la imagen de bordes (binaria).
    """
    # 1) Convertir BGR → HSV y enmascarar rojo
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    lower1 = np.array([0, 100, 50])
    upper1 = np.array([10, 255, 255])
    lower2 = np.array([170, 100, 50])
    upper2 = np.array([180, 255, 255])
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
    # tras: edges = cv2.Canny(…)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
    # 1 px de dilatación para soldar pequeños huecos
    edges = cv2.dilate(edges, kernel, iterations=1)
    # luego un par de closings para cerrar discontinuidades
    edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, iterations=2)

    return edges

def extraer_trayectoria(edges):
    """
    Toma una imagen binaria de bordes (edges) y devuelve un array Nx2
    con las coordenadas (x, y) ordenadas a lo largo del trazado.
    """
    # 1) Esqueletizar para quedarnos con línea de 1-px de ancho
    b = edges > 0
    skel = skeletonize(b)                # binary skeleton

    # 2) Convertir skeleton a uint8 para OpenCV
    skel_u8 = (skel.astype(np.uint8) * 255)

    # 3) Encontrar contornos (píxeles ordenados)
    contours, _ = cv2.findContours(skel_u8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not contours:
        return np.empty((0,2), dtype=int)

    # Elegir el contorno más largo (asumimos que es la pista)
    cont = max(contours, key=lambda c: len(c))
    pts = cont[:,0,:]   # de forma (N,1,2) → (N,2) [x, y]

    return pts

def guardar_csv(pts, filename='trayectoria.csv'):
    """
    Guarda el array Nx2 pts en un CSV con columnas: x, y
    """
    np.savetxt(filename, pts, delimiter=',', header='x,y', comments='', fmt='%d')
    print(f"Trayectoria guardada en {filename} ({len(pts)} puntos).")

def main():
    ruta = 'track.png'
    img = cv2.imread(ruta)
    if img is None:
        print(f"No se pudo cargar la imagen {ruta}")
        return

    # 1) Detectar bordes en la línea roja
    edges = detectar_bordes_linea_roja(img)

    # 2) Extraer trayectoria ordenada
    pts = extraer_trayectoria(edges)
    if pts.size == 0:
        print("No se encontró trayectoria.")
        return

    # 3) Guardar trayectoria a CSV
    guardar_csv(pts, 'trayectoria.csv')

    # 4) Visualizaciones opcionales
    # Overlay de bordes en verde sobre original
    overlay = img.copy()
    overlay[edges != 0] = (0, 255, 0)
    cv2.imshow('Original', img)
    cv2.imshow('Overlay de bordes', overlay)

    # Track sobre fondo negro
    track_negro = np.zeros_like(img)
    track_negro[edges != 0] = (255, 255, 255)
    cv2.imshow('Track sobre fondo negro', track_negro)

    # Skeleton ordenado
    skeleton_img = np.zeros_like(img)
    for x, y in pts:
        skeleton_img[y, x] = (255, 255, 255)
    cv2.imshow('Esqueleto ordenado', skeleton_img)

    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
