## Subcarpeta `python/`

Herramientas mínimas para generar/convertir la trayectoria que usa el simulador MATLAB.


| Requisito | Instalar |
|-----------|----------|
| Python ≥ 3.8 | `pip install opencv-python numpy scikit-image` |

### Uso exprés
```bash
# 1) coloca tu imagen de pista (línea roja) en la carpeta
python3 track_reference.py
# 2) sigue la ventana de calibración de referencia
# 3) obtendrás `trayectoria_m.csv` ⇒ úsalo en finalV6.m
```
