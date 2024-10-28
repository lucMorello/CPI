  /*
  * Docente: Dhiego Fernandes Carvalho
  * Discente: Lucas Martins Morello
  * GitHub: https://github.com/lucMorello
  * Atividade Extra
  * 21/10/2024
  */

const int SENSOR_PIN = 4;  // Pino de entrada analógico (sensor)
const int CONTROL_PIN = 23;  // Pino de saída PWM (controlador)
const int POTENTIOMETER_PIN = 15;    // Pino onde o potenciômetro está conectado

double dt, last_time;  // Variáveis para armazenar o tempo entre amostras e o tempo da última iteração
double integral = 0, previous_error = 0, previous_filtered_derivative = 0, output = 0;  // Variáveis para o cálculo do controlador PID
double kp, ki, kd;  // Ganhos do controlador PID
double setpoint = 200;  // Setpoint inicial (valor desejado)

unsigned long startTime;   // Variável para armazenar o tempo inicial
unsigned long currentTime; // Variável para calcular o tempo atual

bool use_derivative_filter = true;  // Habilitar ou desabilitar o filtro derivativo
bool use_anti_windup = true;  // Habilitar ou desabilitar o anti-windup (limitação do valor integral)
bool use_reference_weighting = true;  // Habilitar ou desabilitar a ponderação do setpoint

double Kbc;  // Ganho de retrocálculo anti-windup
double beta;  // Fator de ponderação proporcional (influência do setpoint na ação proporcional)
double alpha; // Coeficiente de filtro derivativo (para suavizar o termo derivativo)
double K = 1.0;  // Ganho estático do sistema (ajustar conforme necessário)
double tau = 0.5;  // Constante de tempo do sistema (ajustar conforme necessário)

void setup(){
  // Aplicando a sintonia de Skogestad para definir os ganhos do PID
  kp = 1.0 / (K * (tau + 0));  // Skogestad sugere o uso do atraso de transporte theta, que aqui é considerado 0
  ki = kp / tau;               // O ganho integral é o ganho proporcional dividido por tau
  kd = kp * tau / 2;           // O ganho derivativo é o ganho proporcional vezes tau dividido por 2

  last_time = 0;               // Inicializa o tempo da última iteração como 0
  Kbc = ki;                    // Ganho de retrocálculo anti-windup igual ao ganho integral
  beta = 0.5;                  // Ponderação do setpoint no termo proporcional
  alpha = 0.02;                // Coeficiente de filtro derivativo

  Serial.begin(115200);        // Inicializa a comunicação serial com velocidade de 115200 bps
  analogWrite(CONTROL_PIN, 0);  // Inicializa a saída PWM como 0
  startTime = millis();        // Armazena o tempo inicial em milissegundos

  // Loop inicial para imprimir zeros na fase de configuração do sistema
  for (int i = 0; i < 50; i++){
    currentTime = (millis() - startTime) / 1000.0;  // Calcula o tempo decorrido em segundos
    Serial.print(currentTime);  // Imprime o tempo decorrido
    Serial.print(",");
    Serial.println(0);  // Imprime a saída inicial (0) para estabelecer uma linha de base
    delay(100);         // Insere um atraso de 100ms para espaçar as impressões
  }
  delay(100);  // Pequeno atraso adicional para permitir que o sistema estabilize
}

void loop(){
  // Lê o valor do potenciômetro (entre 0 e 1023) e ajusta o setpoint dinamicamente
  setpoint = map(analogRead(POTENTIOMETER_PIN), 0, 1023, 0, 255);  // Ajusta o setpoint entre 0 e 255 (descomentado se o potenciômetro for usado)

  double now = millis();  // Armazena o tempo atual em milissegundos
  dt = (now - last_time) / 1000.00;  // Calcula o intervalo de tempo (dt) em segundos
  last_time = now;  // Atualiza o tempo da última iteração

  // Lê o valor do sensor (de 0 a 1023) e mapeia para a faixa de 0 a 255
  double actual = map(analogRead(SENSOR_PIN), 0, 1023, 0, 255);
  double error = setpoint - actual;  // Calcula o erro (diferença entre setpoint e valor atual)

  // Calcula a saída PID com ponderação de setpoint
  output = pid_skogestad(error, actual, setpoint);  // Passa o valor atual e o setpoint para a função PID

  analogWrite(CONTROL_PIN, output);  // Aplica a saída calculada ao pino de saída PWM

  // Envia o tempo decorrido, setpoint, valor atual, erro e saída para o Plotter Serial
  currentTime = millis() - startTime;  // Calcula o tempo decorrido desde o início em milissegundos
  Serial.print(currentTime / 1000.0);  // Imprime o tempo decorrido em segundos
  Serial.print(",");                   // Separador de vírgula
  Serial.print(setpoint);              // Imprime o valor do setpoint
  Serial.print(",");
  Serial.print(actual);                // Imprime o valor atual do sensor
  Serial.print(",");
  Serial.print(error);                 // Imprime o erro
  Serial.print(",");
  Serial.println(output);              // Imprime o valor de saída PID

  delay(100);  // Insere um atraso no loop para ajustar a taxa de amostragem (opcional)
}

double pid_skogestad(double error, double actual, double setpoint){
  // Termo proporcional com ponderação de setpoint
  double proportional;
  if (use_reference_weighting) {
    proportional = beta * (setpoint - actual);  // Influência ponderada do setpoint
  } else {
    proportional = error;  // Proporcional padrão sem ponderação de setpoint
  }

  // Termo integral
  integral += error * dt;  // Acumula o erro no termo integral

  // Termo derivativo com filtro
  double derivative;
  if (use_derivative_filter) {
    // Aplica o filtro derivativo
    double raw_derivative = (error - previous_error) / dt;
    derivative = alpha * raw_derivative + (1 - alpha) * previous_filtered_derivative;  // Derivada filtrada
    previous_filtered_derivative = derivative;  // Atualiza o valor filtrado para a próxima iteração
  } else {
    // Sem filtro, usa derivada bruta
    derivative = (error - previous_error) / dt;
  }
  
  previous_error = error;  // Atualiza o erro anterior para a próxima iteração

  // Calcula a saída PID com ou sem derivada filtrada
  double output = kp * (proportional + integral * ki + derivative * kd);

  // Anti-windup para limitar a integral se necessário
  if (use_anti_windup) {
    double max_output = 255;  // Valor máximo de saída PWM
    double min_output = 0;    // Valor mínimo de saída PWM
    if (output > max_output) {
      output = max_output;  // Limita a saída ao valor máximo
      integral -= Kbc * (output - max_output) * dt;  // Corrige a integral se o sistema saturar
    } else if (output < min_output) {
      output = min_output;  // Limita a saída ao valor mínimo
      integral -= Kbc * (output - min_output) * dt;  // Corrige a integral se o sistema saturar
    }
  }

  return output;  // Retorna a saída calculada
}