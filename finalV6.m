%% SIMULACIÓN COMPLETA FSAE CON CRITERIO G–G–V + MODELO DE INVERSOR Y MOTOR 
% (CON REGENERACIÓN Y MODELO I–V DE BATERÍA AJUSTADO, curvas estacionarias y animación 2D de endurance)
% Autor: Iván de México Barragán Peimbert
% Versión: 2025-06-02 (límites reales de corriente, pérdidas Joule, regeneración realista y curvas estacionarias)

clc; clear; close all;

%% ------------------------------------------------------------------------
%% Parámetro de habilitación de regeneración (true/false) -----------------
enableRegeneration = true;  % poner true para habilitar regeneración

%% ------------------------------------------------------------------------
%% 0) Crear motorMap.mat solo la primera vez ------------------------------
%   Guarda: omegaVec, TmaxVec, etaVec, R_w, i_g, k_t, R_ph, J_motor, k_e
if ~isfile('motorMap.mat')
    %--------------------------%
    % Parámetros EMRAX 228 MV  %
    %--------------------------%
    R_w       = 0.2286;       % [m] radio efectivo de rueda
    i_g       = 3.5;          % relación total fija
    k_t       = 0.61;         % [Nm/A] constante torque (HV‐CC)
    R_ph      = 7.06e-3;      % [Ω] resistencia fase del motor
    J_motor   = 0.02521;      % [kg·m²] inercia rotor
    k_e       = 0.04793;      % [V_rms/rpm] constante back‐EMF (HV‐CC)
    %--------------------------%
    % Mapas digitalizados (8 puntos) %
    % ω [rad/s] y T_lim [Nm], η [–]
    rpmVec   = [0   500  1500  3000  3500  4500  5500  6500];
    omegaVec = rpmVec * 2*pi/60;                  % [rad/s]
    TmaxVec  = [220 220 220  220   215   190   150   100];  % [Nm]
    etaVec   = [0.90 0.93 0.95 0.96 0.95 0.93 0.90 0.85];    % [–]
    save motorMap omegaVec TmaxVec etaVec R_w i_g k_t R_ph J_motor k_e
    fprintf('motorMap.mat creado con %d puntos.\n', numel(omegaVec));
end

%% ------------------------------------------------------------------------
%% 1) Leer CSV con la trayectoria ----------------------------------------
data = readmatrix('./python/trayectoria_m.csv');  % [x, y] en metros
x    = data(:,1);
y    = data(:,2);
s    = [0; cumsum(hypot(diff(x), diff(y)))] ;  % longitud acumulada [m]

% Interpolar a 1000 puntos uniformes en s
s_u  = linspace(0, s(end), 1000)';              % [m]
x_u  = interp1(s, x, s_u, 'pchip');
y_u  = interp1(s, y, s_u, 'pchip');
fprintf('Longitud total de la trayectoria: %.2f m\n', s(end));

%% ------------------------------------------------------------------------
%% 2) Parámetros globales -------------------------------------------------
m   = 200;           % [kg] masa total con piloto
g   = 9.81;          % [m/s²]

% Neumáticos / adherencia
mu_tire      = 1.8;     % coef. fricción “pura” (slick caliente)
a_long_pos_g = 1.4;     % g-longitudinal máx. en tracción
a_long_neg_g = 1.6;     % g-longitudinal máx. en frenada
k_safety     = 0.90;    % factor de seguridad (reduce límites)

% Motor / energía
P_peak = 80e3;          % [W] potencia pico disponible

% Aerodinámica
rho = 1.18;             % [kg/m³] densidad del aire (~25 °C)
A   = 1.00;             % [m²] área frontal efectiva
Cl  = -1.8;             % coef. sustentación (negativo = downforce)
Cd  = 1.2;              % coef. drag con paquete aerodinámico
Crr = 0.012;            % coef. rodadura slick

% Post-proceso
Lsmooth = 3;            % [m] ventana para suavizar v(s)

%% ------------------------------------------------------------------------
%% 3) Cargar el mapa del motor --------------------------------------------
load motorMap   % carga: omegaVec, TmaxVec, etaVec, R_w, i_g, k_t, R_ph, J_motor, k_e

% Funciones de interpolación motor
omega_fun = @(v) (v / R_w) * i_g;                 % [rad/s] motor desde velocidad vehículo
Tlim_fun  = @(w) interp1(omegaVec, TmaxVec, w, 'pchip', 'extrap');  % [Nm]
eta_mot   = @(w) interp1(omegaVec, etaVec, w, 'pchip', 'extrap');    % [–]

%% ------------------------------------------------------------------------
%% 4) Lectura de tabla de pérdidas del inversor DTI HV-550/HV-850 --------
I_vec_inv  = [100, 200, 300, 400, 500, 600];   % [Arms]
V_vec_inv  = [200, 400, 600, 800];             % [Vdc]
P_loss_tbl = [ 261   384   513   642;    % 100 A
               552   783  1020  1260;    % 200 A
               894  1233  1584  1956;    % 300 A
              1300  1750  2200  2740;    % 400 A
              1740  2300  2900  3550;    % 500 A
              2250  2900  3650  4500 ];  % 600 A

F_loss = griddedInterpolant({I_vec_inv, V_vec_inv}, P_loss_tbl, 'linear', 'linear');
eta_inv_fun = @(P_out, V_dc) arrayfun(@(P,V) ...
    max(0.01, P ./ (P + F_loss(max(min(P/(sqrt(3)*V), 600), 100), V))), ...
    P_out, V_dc);

%% ------------------------------------------------------------------------
%% 5) Celdas Molicel P42A y dimensionamiento inicial ----------------------
cell.V_nom = 3.6;     % [V] nominal celda
cell.Q_Ah  = 4.2;     % [Ah]
cell.R_int = 15e-3;   % [Ω] resistencia interna típica
cell.I_max = 45;      % [A] máxima continua

% Restricción reglamento: pack.V_target ≤ 600 V
pack.V_target = 600;      % [V] pack nominal máximo
Ns = floor(pack.V_target / cell.V_nom);
V_pack_nom = Ns * cell.V_nom;   % [V] pack nominal efectivo
Ns_mod = floor(120 / cell.V_nom);
Nm     = ceil(Ns / Ns_mod);    % Módulos totales (cada ≤120 V)

R_pack = 0;

%% ------------------------------------------------------------------------
%% 6) Curvatura y límite geométrico con downforce -------------------------
dx  = gradient(x_u, s_u);
dy  = gradient(y_u, s_u);
ddx = gradient(dx , s_u);
ddy = gradient(dy , s_u);
kappa = abs(dx.*ddy - dy.*ddx) ./ (dx.^2 + dy.^2).^1.5;  % curvatura κ(s)

v_curve_geom = sqrt(mu_tire * g ./ max(kappa, 1e-6));    % [m/s]
Fz_geom = 0.5 * rho * abs(Cl) * A .* (v_curve_geom.^2);
g_eff   = g + Fz_geom / m;
v_curve = sqrt(mu_tire * g_eff ./ max(kappa, 1e-6));      % [m/s]

%% ------------------------------------------------------------------------
%% 7) Pasada hacia delante (G–G–V combinado: tracción + motor + resist) ----
N       = numel(s_u);
v_fwd   = zeros(N,1);
v_fwd(1) = 0.1;   % [m/s] semilla para arrancar

a_trac_vec = zeros(N,1);
a_tq_vec   = zeros(N,1);
a_pow_vec  = zeros(N,1);
a_lat_vec  = zeros(N,1);
g_eff_vec  = zeros(N,1);
P_batt_fw  = zeros(N,1);
I_ac_fw    = zeros(N,1);

for i = 2:N
    ds      = s_u(i) - s_u(i-1);
    v_prev  = max(v_fwd(i-1), 0.1);

    % --- Lateral ---
    a_lat   = kappa(i) * v_prev^2;
    Fz      = 0.5 * rho * abs(Cl) * A * v_prev^2;
    g_eff_i = g + Fz / m;
    g_eff_vec(i) = g_eff_i;
    a_trac_lat_max = mu_tire * g_eff_i;
    a_lat_vec(i)   = a_lat;

    if a_lat >= a_trac_lat_max
        a_long_comb_max = 0;
    else
        a_long_comb_max = sqrt(a_trac_lat_max^2 - a_lat^2);
    end
    a_trac_vec(i) = a_trac_lat_max;

    % --- Resistencias ---
    F_drag = 0.5 * rho * Cd * A * v_prev^2;
    F_rr   = m * g * Crr;
    P_res  = (F_drag + F_rr) * v_prev;

    % --- Torque rueda ---
    omega_i  = omega_fun(v_prev);
    T_lim_i  = Tlim_fun(omega_i);
    if omega_i > 5500*2*pi/60
        T_lim_cont = (75e3) / omega_i;
        T_lim_i = min(T_lim_i, T_lim_cont);
    end
    F_trac_i = (T_lim_i * i_g) / R_w;
    a_torque = F_trac_i / m;
    a_tq_vec(i) = a_torque;

    % --- Eficiencia motor ---
    I_mot = T_lim_i / k_t;
    I_mot = min(I_mot, 360);
    P_Cu   = 3 * I_mot^2 * R_ph;
    P_out_m = T_lim_i * omega_i;
    P_core  = 0.05 * P_out_m;
    if (P_out_m + P_Cu + P_core) > 0
        eta_mot_i = P_out_m / (P_out_m + P_Cu + P_core);
    else
        eta_mot_i = 1;
    end

    % --- Potencia disponible ---
    P_out_m = min(P_out_m, P_peak);
    P_elec_req = P_out_m / eta_mot_i;

    I_ac_i = P_elec_req / (sqrt(3) * V_pack_nom);
    I_ac_i = min(max(I_ac_i, 0), 360);
    I_ac_fw(i) = I_ac_i;

    P_loss_inv = F_loss(max(min(I_ac_i,600),100), V_pack_nom);
    eta_inv_i   = max(0.01, P_elec_req / (P_elec_req + P_loss_inv));

    P_batt_i    = P_elec_req / eta_inv_i;
    P_batt_fw(i)= P_batt_i;

    if v_prev < 1
        a_power = inf;
    else
        a_power = max(0, (P_out_m - P_res) / (m * v_prev));
    end
    a_pow_vec(i) = a_power;

    a_trac_motor_max = min([k_safety * a_long_pos_g * g, a_torque, a_power]);
    a_plus = min(a_trac_motor_max, a_long_comb_max);

    v_fwd(i) = min(v_curve(i), sqrt(v_prev^2 + 2 * a_plus * ds));
end

%% ------------------------------------------------------------------------
%% 8) Pasada hacia atrás (frenada + resistencias + G–G–V + regenerativo) ---
v_bwd   = v_curve;
v_bwd(end) = 0;
P_batt_bw = zeros(N,1);

for i = N-1:-1:1
    ds      = s_u(i+1) - s_u(i);
    v_next  = v_bwd(i+1);

    a_lat_f = kappa(i) * v_next^2;
    Fz_f    = 0.5 * rho * abs(Cl) * A * v_next^2;
    g_eff_f = g + Fz_f / m;
    a_trac_lat_f_max = mu_tire * g_eff_f;

    F_drag = 0.5 * rho * Cd * A * v_next^2;
    F_rr   = m * g * Crr;
    a_res  = (F_drag + F_rr) / m;

    a_long_brake_max = k_safety * a_long_neg_g * g + a_res;

    if a_lat_f >= a_trac_lat_f_max
        a_long_comb_max_b = 0;
    else
        a_long_comb_max_b = sqrt(a_trac_lat_f_max^2 - a_lat_f^2);
    end

    a_brake_real = min(a_long_brake_max, a_long_comb_max_b);
    v_bwd(i) = min(v_curve(i), sqrt(v_next^2 + 2 * a_brake_real * ds));

    if ~enableRegeneration
        P_batt_bw(i) = 0;
        continue;
    end

    P_brake_mech = m * a_brake_real * v_next;
    P_res_f      = (F_drag + F_rr) * v_next;
    P_reg_mech   = max(0, P_brake_mech - P_res_f);
    P_reg_mech   = min(P_reg_mech, 15e3);
    eta_mot_reg  = 0.80;
    P_elec_gen   = P_reg_mech * eta_mot_reg;
    packOutApprox.Np = Nm;
    I_pack_max_reg   = 0.7 * cell.Q_Ah * packOutApprox.Np;
    I_ac_reg = P_elec_gen / (sqrt(3) * V_pack_nom);
    I_ac_reg = min(I_ac_reg, I_pack_max_reg);
    P_elec_gen = I_ac_reg * sqrt(3) * V_pack_nom;
    eta_inv_nom = eta_inv_fun(P_elec_gen, V_pack_nom);
    eta_inv_reg = 0.90 * eta_inv_nom;
    eta_charge = 0.95;
    P_batt_bw(i) = - P_elec_gen * eta_inv_reg * eta_charge;
end

%% ------------------------------------------------------------------------
%% 9) Perfil final y lap-time --------------------------------------------
v_opt = smoothdata(min(v_fwd, v_bwd), 'movmean', ...
         round(Lsmooth / mean(diff(s_u))));

eps_v = 0.05;
denom = v_opt(1:end-1) + v_opt(2:end);
denom(denom < eps_v) = eps_v;
dt_seg = 2 * diff(s_u) ./ denom;
lap_time = sum(dt_seg);
fprintf('Tiempo total estimado (G–G–V + pérdidas): %.2f s\n\n', lap_time);

t_u = [0; cumsum(dt_seg)];

%% ------------------------------------------------------------------------
%% 10) Cálculo de energía por vuelta (con/sin regeneración) ---------------
P_batt   = zeros(N-1,1);
E_seg_J  = zeros(N-1,1);

for i = 1:N-1
    v_i   = v_opt(i);
    v_ip1 = v_opt(i+1);
    ds_i  = s_u(i+1) - s_u(i);

    a_inst = (v_ip1^2 - v_i^2) / (2 * ds_i);

    if a_inst > 0
        F_drag_i    = 0.5 * rho * Cd * A * v_i^2;
        F_rr_i      = m * g * Crr;
        F_trac_req  = m * a_inst + F_drag_i + F_rr_i;
        omega_i  = omega_fun(v_i);
        T_req_i  = (F_trac_req * R_w) / i_g;
        T_lim_i  = Tlim_fun(omega_i);
        if omega_i > 5500*2*pi/60
            T_lim_cont = (75e3) / omega_i;
            T_lim_i = min(T_lim_i, T_lim_cont);
        end
        T_act_i  = min([T_req_i, T_lim_i, (m * k_safety * a_long_pos_g * g) * (R_w / i_g)]);
        P_out_m  = T_act_i * omega_i;
        I_mot    = min(T_act_i / k_t, 360);
        P_Cu     = 3 * I_mot^2 * R_ph;
        P_core   = 0.05 * P_out_m;
        if (P_out_m + P_Cu + P_core) > 0
            eta_mot_i = P_out_m / (P_out_m + P_Cu + P_core);
        else
            eta_mot_i = 1;
        end
        P_elec_req = P_out_m / eta_mot_i;
        I_ac_i = P_elec_req / (sqrt(3) * V_pack_nom);
        I_ac_i = min(max(I_ac_i, 0), 360);
        P_loss_inv = F_loss(max(min(I_ac_i,600),100), V_pack_nom);
        eta_inv_i   = max(0.01, P_elec_req / (P_elec_req + P_loss_inv));
        P_req_batt_i = P_elec_req / eta_inv_i;
        P_batt(i)    = P_req_batt_i;
    else
        if ~enableRegeneration
            P_batt(i) = 0;
        else
            P_brake_mech = m * abs(a_inst) * v_i;
            P_res_i      = (0.5 * rho * Cd * A * v_i^2 + m * g * Crr) * v_i;
            P_reg_mech   = max(0, P_brake_mech - P_res_i);
            P_reg_mech   = min(P_reg_mech, 15e3);
            omega_i      = omega_fun(v_i);
            eta_mot_reg  = 0.80;
            P_elec_gen   = P_reg_mech * eta_mot_reg;
            packOutApprox.Np = Nm;
            I_pack_max_reg   = 0.7 * cell.Q_Ah * packOutApprox.Np;
            I_ac_reg   = P_elec_gen / (sqrt(3) * V_pack_nom);
            I_ac_reg   = min(I_ac_reg, I_pack_max_reg);
            P_elec_gen = I_ac_reg * sqrt(3) * V_pack_nom;
            eta_inv_nom = eta_inv_fun(P_elec_gen, V_pack_nom);
            eta_inv_reg = 0.90 * eta_inv_nom;
            eta_charge   = 0.95;
            P_batt(i) = - P_elec_gen * eta_inv_reg * eta_charge;
        end
    end

    dt_i       = dt_seg(i);
    E_seg_J(i) = P_batt(i) * dt_i;
end

E_lap_J   = sum(E_seg_J);
E_lap_Wh  = E_lap_J / 3600;
fprintf('Energía neta consumida (1 vuelta): %.2f Wh\n\n', E_lap_Wh);

%% ------------------------------------------------------------------------
%% 11) Dimensionamiento de batería para 22 vueltas de endurance ------------
nLaps     = 22;
E_end_Wh  = E_lap_Wh * nLaps;
t_end_min = (lap_time * nLaps) / 60;

fprintf('Endurance %d vueltas: %.1f Wh   (%.1f min)\n\n', ...
        nLaps, E_end_Wh, t_end_min);

packOut = sizeBattery(E_end_Wh, cell, pack, max(abs(P_batt))/V_pack_nom);
fprintf('--- Dimensionamiento de batería para %d vueltas ---\n', nLaps);
fprintf('  · Celdas en serie (Ns): %d   (V_pack_nom ≈ %.1f V)\n', ...
        packOut.Ns, packOut.V_nom);
fprintf('  · Módulos (≤120 V cada uno): %d\n',      packOut.Nm);
fprintf('  · Celdas en paralelo (Np): %d   (Q_pack ≈ %.1f Ah)\n', ...
        packOut.Np, packOut.Q_Ah);
fprintf('  · Energía pack total:     %.0f Wh\n',   packOut.E_Wh);
fprintf('  · Corriente pico por celda: %.1f A\n',   packOut.I_cell_peak);
fprintf('  · Corriente pico total pack: %.1f A\n\n', max(abs(P_batt))/V_pack_nom);

R_pack = (packOut.Ns * cell.R_int) / packOut.Np;

%% ------------------------------------------------------------------------
%% 12) Resumen en pantalla para el usuario --------------------------------
[v_max, ~] = max(v_opt);
fprintf('\n===== RESUMEN DE SIMULACIÓN =====\n');
fprintf('Velocidad máxima alcanzada (en perfil): %.2f km/h\n', v_max*3.6);
fprintf('Tiempo de vuelta estimado: %.2f s\n', lap_time);
fprintf('Energía neta consumida por vuelta: %.2f Wh\n', E_lap_Wh);
fprintf('Endurance (%d vueltas): %.0f Wh totales  (≈ %.1f min)\n', nLaps, E_end_Wh, t_end_min);
fprintf('\n--- DIMENSIONAMIENTO BATERÍA ---\n');
fprintf('Voltaje nominal pack: %.1f V\n', packOut.V_nom);
fprintf('Celdas en serie (Ns): %d   |  Módulos totales: %d\n', packOut.Ns, packOut.Nm);
fprintf('Celdas en paralelo (Np): %d   |  Capacidad ≈ %.1f Ah\n', packOut.Np, packOut.Q_Ah);
fprintf('Energía total del pack: %.0f Wh\n', packOut.E_Wh);
fprintf('Corriente pico total del pack: %.1f A\n', max(abs(P_batt))/V_pack_nom);
fprintf('Corriente pico por celda: %.1f A\n', packOut.I_cell_peak);
v_aero_limit = ((P_peak/3) / (0.5 * rho * Cd * A))^(1/3);
fprintf('\nVelocidad teórica máxima limitada por la potencia disponible (sin G–G–V): %.2f km/h\n', ...
        v_aero_limit*3.6);
fprintf('========================================\n\n');

%% ------------------------------------------------------------------------
%% 13) Visualización rápida de resultados (subplots en 3 ventanas) --------

% 13.1 Ventana 1: Velocidad vs distancia y Aceleraciones vs distancia
figure('Name','Velocidad y Aceleraciones vs Distancia');
% Subplot 1: Velocidad vs s
subplot(2,1,1);
plot(s_u, v_opt, 'LineWidth', 1.3); grid on;
xlabel('s [m]'); ylabel('v [m/s]');
title('Perfil de velocidad (1 vuelta)');

% Subplot 2: Aceleraciones longitudinal y lateral vs s
subplot(2,1,2);
a_long_opt = [diff(v_opt.^2)./(2*diff(s_u)); 0];
a_lat_opt  = kappa .* v_opt.^2;
plot(s_u, a_long_opt / g, 'LineWidth', 1.3); hold on; grid on;
plot(s_u, a_lat_opt  / g, '--', 'LineWidth', 1.3);
xlabel('s [m]'); ylabel('a [g]');
legend('a_{long}','a_{lat}','Location','best');
title('Aceleraciones longitudinal y lateral');

% 13.2 Ventana 2: Potencia neta pedida, Energía acumulada, SoC y Corriente motor
figure('Name','Potencia, Energía, SoC y Corriente vs Tiempo/Distancia');
% Subplot 1: Potencia pedida a batería vs s
subplot(2,2,1);
plot(s_u(1:end-1), P_batt/1e3, 'LineWidth', 1.3); grid on;
xlabel('s [m]'); ylabel('P_{batt} [kW]');
title('Potencia neta pedida a batería vs distancia');

% Subplot 2: Energía acumulada vs tiempo
subplot(2,2,2);
E_cum_J = cumsum(E_seg_J);
E_cum_Wh = E_cum_J / 3600;
plot(t_u(1:end-1), E_cum_Wh, 'LineWidth', 1.3); grid on;
xlabel('t [s]'); ylabel('E_{acum} [Wh]');
title('Energía acumulada vs tiempo');

% Subplot 3: SoC vs tiempo
subplot(2,2,3);
SoC = 100 * max(0, 1 - E_cum_Wh / packOut.E_Wh);
plot(t_u(1:end-1), SoC, 'LineWidth', 1.3); grid on;
xlabel('t [s]'); ylabel('SoC [%]');
title('Estado de carga vs tiempo');

% Subplot 4: Corriente AC motor vs tiempo (tracción)
subplot(2,2,4);
plot(t_u, I_ac_fw, 'LineWidth', 1.3); grid on;
xlabel('t [s]'); ylabel('I_{AC} [A]');
title('Corriente del motor vs tiempo (tracción)');

% 13.3 Ventana 3: Curvas estacionarias (Torque, Fuerza, Aceleración, Potencia y Corriente)
% 14.1 Rango velocidades estacionarias (0–30 m/s)
np = 300;
v_plot   = linspace(0, 30, np);
v_kmh    = v_plot * 3.6;

F_trac_v = zeros(1,np);
P_batt_v = zeros(1,np);
I_pack_v = zeros(1,np);
a_motor_v= zeros(1,np);
a_ggv_v  = zeros(1,np);

kappa_min = min(kappa);

for j = 1:np
    vj = v_plot(j);
    omega_j = omega_fun(vj);
    Tj = interp1(omegaVec, TmaxVec, omega_j, 'pchip', 'extrap');
    if omega_j > 5500*2*pi/60
        T_cont = (75e3)/omega_j;
        Tj = min(Tj, T_cont);
    end

    F_trac_v(j) = (Tj * i_g) / R_w;

    Ij      = min(Tj / k_t, 360);
    P_Cu_j  = 3 * Ij^2 * R_ph;
    P_out_j = Tj * omega_j;
    P_core_j= 0.05 * P_out_j;
    if (P_out_j + P_Cu_j + P_core_j) > 0
        eta_mot_j = P_out_j / (P_out_j + P_Cu_j + P_core_j);
    else
        eta_mot_j = 1;
    end
    P_elec_j = P_out_j / eta_mot_j;

    Iac_j = P_elec_j / (sqrt(3) * V_pack_nom);
    Iac_j = min(max(Iac_j, 100), 600);
    P_loss_inv_j = F_loss(Iac_j, V_pack_nom);
    eta_inv_j     = max(0.01, P_elec_j / (P_elec_j + P_loss_inv_j));

    P_batt_v(j) = P_elec_j / eta_inv_j;
    I_pack_v(j) = P_batt_v(j) / V_pack_nom;

    a_mot_j = F_trac_v(j) / m;
    a_mot_j = min(a_mot_j, k_safety * a_long_pos_g * g);
    a_motor_v(j) = a_mot_j;

    a_lat_j = kappa_min * vj^2;
    Fz_j   = 0.5 * rho * abs(Cl) * A * vj^2;
    g_eff_j= g + Fz_j / m;
    a_trac_lat_j = mu_tire * g_eff_j;
    if a_lat_j >= a_trac_lat_j
        a_ggv_v(j) = 0;
    else
        a_ggv_v(j) = sqrt(a_trac_lat_j^2 - a_lat_j^2);
    end
end

% 14.2 Torque y Potencia vs RPM
omega_plot = linspace(0, max(omega_fun(30))*1.1, 400);
T_lim_plot = arrayfun(@(w) ...
    min(interp1(omegaVec, TmaxVec, w, 'pchip', 'extrap'), (75e3)/w), ...
    omega_plot);
P_mech_plot = T_lim_plot .* omega_plot;

figure('Name','Curvas Estacionarias');
% Subplot 1: Mapa Motor (T vs RPM y P vs RPM)
subplot(3,2,1);
yyaxis left
plot(omega_plot * 60/(2*pi), T_lim_plot, 'LineWidth', 1.5);
ylabel('Torque [Nm]');
yyaxis right
plot(omega_plot * 60/(2*pi), P_mech_plot/1e3, '--', 'LineWidth', 1.5);
ylabel('Potencia mecánica [kW]');
xlabel('Velocidad del motor [rpm]');
yline(80, 'r:', '80 kW límite continuo');
grid on;
title('Mapa T–ω y P–ω Motor EMRAX 228');
legend('T_{lim}','P_{mech}','80 kW','Location','best');

% Subplot 2: Fuerza de tracción vs velocidad
subplot(3,2,2);
plot(v_kmh, F_trac_v, 'LineWidth', 1.3); grid on;
xlabel('v [km/h]'); ylabel('F_{trac} [N]');
title('Fuerza de tracción vs velocidad (estacionario)');

% Subplot 3: Aceleraciones vs velocidad
subplot(3,2,3);
plot(v_kmh, a_motor_v/g, 'b-',  'LineWidth', 1.3); hold on;
plot(v_kmh, a_ggv_v/g,   'r--', 'LineWidth', 1.3);
plot(v_kmh, min(a_motor_v,a_ggv_v)/g, 'k-.', 'LineWidth', 1.5);
grid on;
xlabel('v [km/h]'); ylabel('a [g]');
legend('a_{motor}','a_{G–G–V}','a_{final}','Location','best');
title('Aceleración vs velocidad (estacionario)');

% Subplot 4: P_batt vs velocidad (estacionario)
subplot(3,2,4);
plot(v_kmh, P_batt_v/1e3, 'LineWidth', 1.3); grid on;
xlabel('v [km/h]'); ylabel('P_{batt} [kW]');
title('Potencia neta pedida a batería vs velocidad');

% Subplot 5: Corriente pack vs velocidad (estacionario)
subplot(3,2,5);
plot(v_kmh, I_pack_v, 'LineWidth', 1.3); grid on;
xlabel('v [km/h]'); ylabel('I_{pack} [A]');
title('Corriente pack vs velocidad');

% Sexto espacio libre para mantener la rejilla
subplot(3,2,6);
axis off;

%% ------------------------------------------------------------------------
%% 15) Animación 2D de ENDURANCE (nLaps vueltas) --------------------------
playbackFactor = 1;  % 1 = tiempo real, 2 = x2
saveVideo = true;

fig = figure('Name','Endurance Animation G–G–V','Color','w');
ax  = axes(fig); hold(ax,'on'); axis(ax,'equal');
plot(ax, x_u, y_u, 'k', 'LineWidth', 1);  % contorno pista
grid(ax,'on');
xlabel(ax,'X [m]'); ylabel(ax,'Y [m]');
title(ax,'Vehículo recorriendo multiple vueltas (Endurance)');

vehPoint = plot(ax, x_u(1), y_u(1), 'ro', ...
                'MarkerFaceColor','r', 'MarkerSize', 6);

if saveVideo
    vidName = 'EnduranceAnimation_GGV.avi';
    vid = VideoWriter(vidName, 'Motion JPEG AVI');
    vid.FrameRate = 30;    % fps fijo
    open(vid);
    writeVideo(vid, getframe(ax));  % primer cuadro fija dimensiones
end

for lap = 1:nLaps
    for k = 1:N
        set(vehPoint, 'XData', x_u(k), 'YData', y_u(k));
        drawnow limitrate;
        if saveVideo
            writeVideo(vid, getframe(ax));
        end
        if k < N
            pause(dt_seg(k) / playbackFactor);
        end
    end
end

if saveVideo
    close(vid);
    if isfile(vidName)
        fprintf('✅ Vídeo guardado: %s\n', vidName);
    else
        warning('⚠️ No se encontró %s tras cerrar el vídeo.', vidName);
    end
end

%% ------------------------------------------------------------------------
%% Función auxiliar: dimensionamiento de batería (pack ≤600 V) ------------
function out = sizeBattery(E_req_Wh, cell, pack, I_peak_pack)
    Ns = floor(pack.V_target / cell.V_nom);
    if Ns < 1
        error('pack.V_target demasiado bajo para la celda seleccionada.');
    end
    V_nom = Ns * cell.V_nom;

    Ns_mod = floor(120 / cell.V_nom);
    Nm = ceil(Ns / Ns_mod);

    Q_req_Ah = E_req_Wh / V_nom;
    Q_req_Ah = Q_req_Ah * 1.10;
    Np_E = ceil(Q_req_Ah / cell.Q_Ah);
    Np_I = ceil(1.2 * I_peak_pack / cell.I_max);
    Np   = max(Np_E, Np_I);

    out.Nm        = Nm;
    out.Ns        = Ns;
    out.Np        = Np;
    out.V_nom     = V_nom;
    out.Q_Ah      = cell.Q_Ah * Np;
    out.E_Wh      = V_nom * out.Q_Ah;
    out.I_cell_peak = I_peak_pack / Np;
end
