# 🚗 Vehicle-Simulation – Tren Motriz Eléctrico FSAE en MATLAB

Simulador completo y validado del sistema de potencia eléctrica para un monoplaza FSAE, desarrollado para el equipo **FSAE Mty** como herramienta de diseño, análisis y dimensionamiento en etapas tempranas del tren motriz. Esta plataforma permite simular trayectorias tipo *endurance*, evaluar potencia, eficiencia, regeneración y calcular el dimensionamiento óptimo del paquete de baterías bajo restricciones reales del reglamento FSAE.

> Versión actual: `2025-06-02`  
> Autor principal: **Iván de México Barragán Peimbert**  
> Colaborador técnico: **Víctor Misael Escalante Alvarado**  
> Repositorio oficial: [github.com/FSAEMTY/Vehicle-Simulation](https://github.com/FSAEMTY/Vehicle-Simulation)

---

## 🎯 Justificación del Proyecto

El diseño eficiente del sistema de potencia (motor, inversor y batería) en un vehículo FSAE eléctrico requiere validar rendimiento, energía consumida, corrientes pico y factibilidad térmica antes de construir. Esta simulación permite:

- Validar pruebas clave como **endurance**, **aceleración** y **curvas estacionarias**
- Calcular energía por vuelta y dimensionamiento mínimo de batería
- Visualizar esfuerzos dinámicos sobre el sistema eléctrico
- Evaluar escenarios con y sin regeneración

---

## 📌 Importancia para el equipo FSAE Mty

Para el equipo de electrónica, esta herramienta:

- Define corrientes máximas esperadas para selección de sensores y protecciones
- Determina número mínimo de celdas necesarias para 22 vueltas de endurance
- Evalúa SoC restante tras la prueba para definir margen operativo
- Permite visualizar desempeño en animaciones y gráficas físicas

---

## 🚀 Funcionalidades destacadas

- 🔋 Modelo paramétrico de batería con curva I–V y resistencias internas
- ⚡ Motor EMRAX 228 MV con interpolación de torque y eficiencia
- 🧠 Perfil de conducción basado en criterio **G–G–V combinado**
- 🔁 Regeneración de energía opcional (limitada a 15 kW)
- 📊 Curvas estacionarias: torque, aceleración, potencia, corriente
- 🎥 Animación del recorrido en pista tipo OpenLap (hasta 22 vueltas)
- 📈 Visualización automática de potencia, SoC, energía y perfil de velocidad

---

## 📂 Estructura del repositorio

```
Vehicle-Simulation/
├── finalV6.m                 # Script principal (simulación completa)
├── motorMap.mat              # Mapa del motor EMRAX 228
├── EnduranceAnimation_GGV.avi# Animación generada (22 vueltas)
├── pista_fsae.png            # Imagen de la pista
├── python/
│   ├── trayectoria.csv       # Trayectoria tipo autocross / endurance
│   └── track_generator.py    # Scripts opcionales de generación de pista
└── README.md                 # Este archivo
```


---

## ⚙️ Parámetros configurables

### 🎯 Objetivos de desempeño
| Parámetro             | Descripción                                         | Valor por defecto       |
|----------------------|-----------------------------------------------------|--------------------------|
| `nLaps`              | Vueltas de endurance a simular                      | `22`                    |
| `Lsmooth`            | Ventana de suavizado para perfil de velocidad       | `3` [m]                 |
| `playbackFactor`     | Velocidad de animación de video                     | `1` (tiempo real)       |
| `saveVideo`          | Guardar animación en AVI                            | `true`                  |

### 🔋 Batería (Molicel P42A)
| Parámetro         | Descripción                                 | Valor        |
|------------------|---------------------------------------------|--------------|
| `cell.V_nom`     | Voltaje nominal por celda                    | `3.6 V`      |
| `cell.Q_Ah`      | Capacidad nominal por celda                  | `4.2 Ah`     |
| `cell.R_int`     | Resistencia interna típica                   | `15 mΩ`      |
| `cell.I_max`     | Corriente continua máxima por celda          | `45 A`       |
| `pack.V_target`  | Voltaje objetivo del pack (máx. 600V)        | `600 V`      |

### ⚡ Motor EMRAX 228 MV
| Parámetro       | Descripción                          | Valor         |
|----------------|--------------------------------------|---------------|
| `k_t`          | Constante de torque                   | `0.61 Nm/A`   |
| `k_e`          | Constante de back-EMF                 | `0.04793 V/rpm` |
| `R_ph`         | Resistencia de fase                   | `7.06 mΩ`     |
| `J_motor`      | Inercia rotacional                    | `0.02521 kg·m²` |
| `i_g`          | Relación de transmisión total         | `3.5`         |
| `R_w`          | Radio efectivo de rueda               | `0.2286 m`    |
| `P_peak`       | Potencia pico disponible              | `80 kW`       |

### ⚙️ Vehículo y entorno
| Parámetro        | Descripción                           | Valor          |
|-----------------|----------------------------------------|----------------|
| `m`             | Masa total (con piloto)                | `200 kg`       |
| `A`             | Área frontal efectiva                  | `1.00 m²`      |
| `Cd`            | Coeficiente de arrastre                | `1.2`          |
| `Cl`            | Coeficiente de sustentación (negativo) | `-1.8`         |
| `Crr`           | Coeficiente de rodadura                | `0.012`        |
| `rho`           | Densidad del aire                      | `1.18 kg/m³`   |
| `mu_tire`       | Coef. fricción máx. (slick caliente)   | `1.8`          |
| `a_long_pos_g`  | Aceleración máxima (tracción)          | `1.4 g`        |
| `a_long_neg_g`  | Aceleración máxima (frenado)           | `1.6 g`        |
| `k_safety`      | Factor de seguridad dinámico           | `0.90`         |

### 🔁 Regeneración
| Parámetro              | Descripción                                       | Valor         |
|------------------------|---------------------------------------------------|---------------|
| `enableRegeneration`   | Activar regeneración de frenado                   | `true`        |
| `P_reg_max`            | Potencia regenerativa máxima                      | `15 kW`       |
| `eta_mot_reg`          | Eficiencia en modo generador                      | `0.80`        |
| `eta_inv_reg`          | Eficiencia estimada del inversor durante carga    | `90% nominal` |
| `eta_charge`           | Eficiencia estimada del proceso de carga          | `0.95`        |

---

## ▶️ Ejecución rápida

1. Abre MATLAB y ubica el archivo `finalV6.m`.
2. Ejecuta:

```matlab
run('finalV6.m')
```

## 📈 Resultados esperados
![image](https://github.com/user-attachments/assets/bde6b1ae-67ea-490e-b5db-62334bf89347)


---

## 🖼️ Visualizaciones y figuras

### 📊 Figura 1: Velocidad y Aceleración vs Distancia
![image](https://github.com/user-attachments/assets/d1d40ce1-35f6-4999-a5cb-e4045da3960f)


### 📈 Figura 2: Potencia y SoC vs tiempo
> **Visualización de carga, corriente y consumo**
![image](https://github.com/user-attachments/assets/9d018ca5-f0c8-4e99-bd62-1b7815e6fbed)

### 📈 Figura 3: Curvas Estacionarias
![image](https://github.com/user-attachments/assets/8acbb367-276b-4b62-8f8f-70f81a0fbf18)


### 🎥 GIF de simulación *Endurance*
> **Animación del recorrido del vehículo durante 1 vuelta**

![LapAnimation_GGV(1)](https://github.com/user-attachments/assets/fc3a70b1-3509-44bc-a8d8-52638beeb407)


---
## ▶️ Guía Rápida de Uso
1. **Clonar Repositorio**
2. **Abrir en Matlab**
3. **Revisar que esté la carpeta correcta abierta**
4. **Darle Run**
---

## 📚 Referencias utilizadas

1. Hayes, A. (2022). *Electric Powertrain – Energy Systems, Power Electronics, and Drives for Hybrid, Electric and Fuel Cell Vehicles.* Wiley.  
2. IRJET (2022). *Evaluate Traction Forces and Torque for EVs Using MATLAB Simulink.*  
3. Doyle *et al.* (2019). *Lap Time Simulation Tool for the Development of an Electric Formula Student Car.* SAE Technical Paper.  
4. MathWorks (2023). *FSAE ePowertrain Modeling Example* — enlace.

---

## 📜 Licencia

Uso académico para fines de diseño y simulación FSAE.  
Para uso externo, contactar a los autores y dar crédito correspondiente.  
Licencia abierta sin fines comerciales.

---

## 👥 Autores

**Iván de México Barragán Peimbert**  
*Diseño, modelado, simulación y documentación técnica*  

**Víctor Misael Escalante Alvarado**  
*Colaboración en validación técnica y análisis de resultados*  

<sub>Desarrollado para **FSAE Mty** como parte del proyecto ExpogIngenierías 2025 del Tec de Monterrey.  
Este simulador permite una aproximación técnica, justificable y reproducible al diseño del sistema de potencia eléctrico para competencias FSAE.</sub>

