// Definição de pinos
const int motorPin1 = 9; // Pino IN1 no L298N
const int motorPin2 = 8; // Pino IN2 no L298N
const int enablePin = 10; // Pino PWM para controlar a velocidade

const int encoderPin = 2; // Pino do sensor TCRT5000 (interrupção)

// Variáveis de controle do PID
float setpoint = 100; // Velocidade desejada em RPM
float Kp = 1.2, Ki = 0.01, Kd = 0.01; // Ganhos do PID
float pidOutput = 0;
float lastError = 0, integral = 0;

// Variáveis do filtro de Kalman
float xh[3] = {0, 0, 0}; // Estados estimados
float P[3][3] = {{100, 0, 0}, {0, 100, 0}, {0, 0, 100}}; // Matriz P
float Q[3][3] = {{1.2, 0, 0}, {0, 1.2, 0}, {0, 0, 1.2}}; // Matriz Q
float R = 0.5; // Covariância do ruído de medida
float A[3][3] = {{0, 1, 0}, {0, 0, 1}, {-4.23, -9.28, -2.14}};
float B[3] = {0, 0, 4.23};
float H[3] = {1, 0, 0}; // Medição

// Variáveis para leitura do encoder
volatile int encoderCount = 0;
unsigned long lastTime = 0;
float measuredRPM = 0;

// Função de interrupção para o encoder
void encoderISR() {
  encoderCount++;
}

// Configurações iniciais
void setup() {
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(enablePin, OUTPUT);
  
  pinMode(encoderPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderPin), encoderISR, RISING);
  
  Serial.begin(9600);
}

// Loop principal
void loop() {
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= 1000) { // Atualiza a cada 1 segundo
    lastTime = currentTime;
    measuredRPM = (encoderCount / 20.0) * 60.0; // Converte pulsos em RPM (ajustar 20.0 de acordo com o número de pulsos por rotação)
    encoderCount = 0;

    // Filtro de Kalman
    float u = pidOutput; // Entrada do controle (PID)
    kalmanFilter(u, measuredRPM);

    // PID Controller
    float error = setpoint - xh[0]; // Erro baseado no valor estimado pelo filtro Kalman
    integral += error;
    float derivative = error - lastError;
    pidOutput = Kp * error + Ki * integral + Kd * derivative;
    lastError = error;

    // Controla a direção e velocidade do motor
    controlMotor(pidOutput);
  }
}

// Função do filtro de Kalman
void kalmanFilter(float u, float z) {
  // Atualiza o estado esperado
  for (int i = 0; i < 3; i++) {
    xh[i] = A[i][0] * xh[0] + A[i][1] * xh[1] + A[i][2] * xh[2] + B[i] * u;
  }

  // Estimativa do erro de covariância
  float Ptemp[3][3];
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      Ptemp[i][j] = A[i][0] * P[0][j] + A[i][1] * P[1][j] + A[i][2] * P[2][j];
    }
  }
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      P[i][j] = Ptemp[i][j] + Q[i][j];
    }
  }

  // Ganho de Kalman
  float K[3];
  float S = H[0] * P[0][0] * H[0] + R;
  for (int i = 0; i < 3; i++) {
    K[i] = P[i][0] * H[0] / S;
  }

  // Atualização dos estados
  float y = z - H[0] * xh[0]; // Inovação
  for (int i = 0; i < 3; i++) {
    xh[i] = xh[i] + K[i] * y;
  }

  // Atualização da matriz de covariância P
  float KH[3][3];
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      KH[i][j] = K[i] * H[j];
    }
  }
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      P[i][j] = P[i][j] - KH[i][j];
    }
  }
}

// Função para controlar o motor
void controlMotor(float speed) {
  if (speed > 0) {
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
  } else {
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
    speed = -speed; // Inverte o sinal da velocidade
  }
  analogWrite(enablePin, constrain(speed, 0, 255)); // Controla o PWM
}

