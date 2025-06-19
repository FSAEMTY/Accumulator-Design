import cv2
import numpy as np
from skimage.morphology import skeletonize

def calibrar_escala(img):
    """
    Permite al usuario dibujar con dos clicks una línea de referencia
    y captura su distancia real en metros para calcular escala [m/pixel].
    """
    win_name = 'Calibracion'
    pts = []

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(pts) < 2:
            pts.append((x, y))

    # 1) Crear ventana y mostrar imagen
    cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
    cv2.imshow(win_name, img)
    cv2.waitKey(1)  # fuerza la creación

    # 2) Asociar callback
    cv2.setMouseCallback(win_name, on_mouse)

    print("→ Haz click en dos puntos de distancia conocida. Presiona 's' o Enter para confirmar.")
    while True:
        vis = img.copy()
        if len(pts) >= 1:
            cv2.circle(vis, pts[0], 5, (0,255,0), -1)
        if len(pts) == 2:
            cv2.circle(vis, pts[1], 5, (0,255,0), -1)
            cv2.line(vis, pts[0], pts[1], (0,255,0), 2)
        cv2.imshow(win_name, vis)

        key = cv2.waitKey(10) & 0xFF
        if len(pts) == 2 and (key == ord('s') or key == 13):
            break
        if key == ord('q'):
            cv2.destroyWindow(win_name)
            raise RuntimeError("Calibración cancelada.")

    cv2.destroyWindow(win_name)

    (x1, y1), (x2, y2) = pts
    dpix = np.hypot(x2 - x1, y2 - y1)
    real = float(input(f"Distancia en píxeles = {dpix:.2f}. ¿Cuál es esa distancia en metros? "))
    escala = real / dpix
    print(f"→ Escala: {escala:.6f} m/pixel")
    return escala

def detectar_bordes_linea_roja(img_bgr):
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    lower1, upper1 = np.array([0,100,50]), np.array([10,255,255])
    lower2, upper2 = np.array([170,100,50]), np.array([180,255,255])
    m1 = cv2.inRange(hsv, lower1, upper1)
    m2 = cv2.inRange(hsv, lower2, upper2)
    mask_red = cv2.bitwise_or(m1, m2)

    rojo = cv2.bitwise_and(img_bgr, img_bgr, mask=mask_red)
    gray = cv2.cvtColor(rojo, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5,5), 0)
    edges = cv2.Canny(blur, 50, 150, apertureSize=3)

    k = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
    edges = cv2.dilate(edges, k, iterations=1)
    edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, k, iterations=2)
    return edges

def extraer_trayectoria(edges):
    """
    Usa skeleton + findContours para extraer lista de (x,y) píxel ordenados.
    """
    b = edges > 0
    skel = skeletonize(b).astype(np.uint8) * 255
    contours, _ = cv2.findContours(skel, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not contours:
        return np.empty((0,2), dtype=int)
    cont = max(contours, key=lambda c: len(c))
    pts = cont[:,0,:]  # (N,1,2) -> (N,2)
    return pts

def guardar_csv(pts, escala,
               fname_pix='trayectoria_pix.csv',
               fname_m='trayectoria_m.csv'):
    # coordenadas en píxeles
    np.savetxt(fname_pix, pts, delimiter=',',
               header='x_px,y_px', comments='', fmt='%d')
    # coordenadas en metros
    pts_m = pts.astype(float) * escala
    np.savetxt(fname_m, pts_m, delimiter=',',
               header='x_m,y_m', comments='', fmt='%.6f')
    print(f"Guardados {len(pts)} puntos en:\n - {fname_pix}\n - {fname_m}")

def main():
    ruta = 'track.png'
    img = cv2.imread(ruta)
    if img is None:
        print(f"No se pudo cargar la imagen {ruta}")
        return

    # 0) Calibrar escala
    escala = calibrar_escala(img)

    # 1) Detectar bordes y extraer trayectoria
    edges = detectar_bordes_linea_roja(img)
    pts = extraer_trayectoria(edges)
    if pts.size == 0:
        print("No se encontró trayectoria.")
        return

    # 2) Guardar CSV con pixeles y metros
    guardar_csv(pts, escala)

    # 3) Mostrar visualizaciones
    overlay = img.copy()
    overlay[edges != 0] = (0,255,0)
    track_negro = np.zeros_like(img)
    track_negro[edges != 0] = (255,255,255)
    skeleton_img = np.zeros_like(img)
    for x,y in pts:
        skeleton_img[y,x] = (255,255,255)

    cv2.imshow('Original', img)
    cv2.imshow('Overlay de bordes', overlay)
    cv2.imshow('Track sobre fondo negro', track_negro)
    cv2.imshow('Esqueleto ordenado', skeleton_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
