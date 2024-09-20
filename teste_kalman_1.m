% Simulação de Controle de Motor com PID e Filtro de Kalman

clc;
clear;
close all;

% Parâmetros do sistema
setpoint = 100; % RPM desejado
N = 400; % Número de steps
Ts = 0.1; % Tempo de amostragem

% Matriz de estados (modelo discreto)
A = [0, 1, 0; 0, 0, 1; -4.23, -9.28, -2.14];
B = [0; 0; 4.23];
H = [1, 0, 0];
Ad = eye(3) + A*Ts; % Discretização de A
Bd = B*Ts;
Hd = H;

% Inicialização do filtro de Kalman
xh = [0; 0; 0]; % Estado inicial estimado
P = 100*eye(3); % Covariância inicial
Q = 1.2*eye(3); % Covariância do processo
R = 0.5; % Covariância do ruído de medida
K = [1; 1; 1]; % Ganho de Kalman

% Parâmetros PID
Kp = 1.2;
Ki = 0.01;
Kd = 0.01;
integral = 0;
lastError = 0;

% Variáveis para plotagem
time = (0:N-1) * Ts;
measuredRPM = zeros(1, N);
filteredRPM = zeros(1, N);
controlSignal = zeros(1, N);
outputRPM = zeros(1, N);

% Simulação do sistema com dados fictícios de RPM
for k = 2:N
    % Sinal de controle (PID)
    error = setpoint - xh(1);
    integral = integral + error * Ts;
    derivative = (error - lastError) / Ts;
    pidOutput = Kp * error + Ki * integral + Kd * derivative;
    lastError = error;
    
    % Simulação do sistema (ruído e processo fictício)
    w = 0.0367 * (rand() - 0.5); % Ruído do processo
    v = 0.0231 * (rand() - 0.5); % Ruído de medida
    outputRPM(k) = outputRPM(k-1) + Ts * pidOutput + w; % RPM simulado (planta)
    measuredRPM(k) = outputRPM(k) + v; % RPM medido com ruído

    % Filtro de Kalman
    % Predição
    xh = Ad * xh + Bd * pidOutput;
    P = Ad * P * Ad' + Q;
    
    % Atualização (medida)
    S = H * P * H' + R;
    K = P * H' / S;
    xh = xh + K * (measuredRPM(k) - H * xh);
    P = P - K * H * P;

    % Armazena valores filtrados
    filteredRPM(k) = xh(1);
    controlSignal(k) = pidOutput;
end

% Plotagem dos resultados
figure;
subplot(3,1,1);
plot(time, measuredRPM, 'r', 'DisplayName', 'Measured RPM');
hold on;
plot(time, filteredRPM, 'b', 'DisplayName', 'Filtered RPM (Kalman)');
plot(time, setpoint*ones(1,N), 'g--', 'DisplayName', 'Setpoint');
title('Measured vs. Filtered RPM');
xlabel('Time (s)');
ylabel('RPM');
legend;

subplot(3,1,2);
plot(time, controlSignal, 'm');
title('Control Signal (PID Output)');
xlabel('Time (s)');
ylabel('Control Signal');

subplot(3,1,3);
plot(time, outputRPM, 'k', 'DisplayName', 'Output RPM');
title('Actual Motor RPM');
xlabel('Time (s)');
ylabel('RPM');
legend;
