/**
 * @file controle_pid_peltier.ino
 * @brief Sistema de controle PID para resfriamento/aquecimento usando o Peltier com TPS54200
 * 
 * @details Este código implementa um controlador PID completo para manter a temperatura
 * de um sensor PICO640 em um valor alvo (25°C) usando um Peltier acionado por um driver
 * TPS54200 via PWM de 10kHz.
 * 
 * @section hardware Hardware Utilizado
 * - Arduino Uno
 * - Sensor de temperatura PICO640 (leitura analógica)
 * - Peltier 
 * - TPS54200 LED driver
 * - PWM Timer1 (pino 9 do arduino) configurado para 10kHz
 * 
 * @section control Lógica de Controle
 * - Erro positivo (temperatura alta) → Peltier RESFRIA (OCR diminui)
 * - Erro negativo (temperatura baixa) → Peltier DESLIGA (OCR = OFF)
 * - Polaridade invertida devido à conexão física dos fios
 * 
 * @author Raquel Belchior  
 * @date 2025
 * @version 1.0
 */

// ============================================================================
// CONFIGURAÇÃO DE HARDWARE
// ============================================================================

/// @brief Pino de saída PWM conectado ao TPS54200 (Timer1)
const int PIN_PWM_TPS = 9;

/// @brief Pino analógico conectado ao sensor de temperatura PICO640
const int pinVTEMP = A0;

// ============================================================================
// PARÂMETROS DO CONTROLE PID
// ============================================================================

/// @brief Temperatura alvo do sistema em graus Celsius
const float TEMP_ALVO = 25.0;

/**
 * @brief Ganho proporcional (Kp),  Ganho integral (Ki) e Ganho derivativo (Kd) do controlador PID
 * 
 * @details Controla a resposta ao erro ATUAL. Quanto maior, mais agressiva
 * a resposta imediata a desvios de temperatura.
 * @details Controla a resposta ao erro ACUMULADO ao longo do tempo.
 * Elimina erro residual (offset) que o termo P sozinho não consegue corrigir.
* @details Controla a resposta à TAXA DE VARIAÇÃO do erro. Age como um
 * amortecedor, prevendo a tendência e evitando overshoots.
 */
const float Kp = 50.0;
const float Ki = 2.0;
const float Kd = 15.0;

/**
 * @brief Limite anti-windup para o termo integral
 * 
 * @details Evita que o termo integral acumule valores excessivos quando o
 * sistema está saturado (erro grande por muito tempo). Sem este limite,
 * o sistema pode demorar muito para se recuperar de grandes perturbações.
 * 
 * @note Valor em unidades de controle (mesmo que a saída do PID)
 */
const float INTEGRAL_LIMITE = 80.0;

/**
 * @brief Flag de inversão de polaridade da célula Peltier
 * 
 * @details TRUE: Inverte o sinal de controle (fios do Peltier conectados invertidos)
 *          FALSE: Usa sinal de controle normal
 * 
 * @warning Se o Peltier aquece quando deveria resfriar, altere para true
 * 
 * @note Com inversão = true:
 *       - Controle positivo (temp alta) → OCR diminui → Peltier RESFRIA
 *       - Controle negativo (temp baixa) → OCR aumenta → Peltier DESLIGA
 */
const bool INVERTER_POLARIDADE = true;

/**
 * @brief Intervalo entre atualizações do controle PID em milissegundos
 * 
 * @details Define a taxa de amostragem do sistema. Valor menor = resposta
 * mais rápida, mas pode amplificar ruído. Valor maior = mais estável, mas
 * resposta mais lenta.
 * 
 * @note 200ms = 5 atualizações por segundo (5 Hz)
 */
const unsigned long INTERVALO_CONTROLE = 200;

// ============================================================================
// CALIBRAÇÃO DO SENSOR PICO640
// ============================================================================

/**
 * @brief Calibração do PICO640
 * 
 * @details Dados obtidos pelo Datasheet
 */
#define COEF_ANGULAR -207.9
#define VTEMP_REF 2.158
#define TEMP_REF 21.94

// ============================================================================
// CONFIGURAÇÃO DO ADC (CONVERSOR ANALÓGICO-DIGITAL)
// ============================================================================

/// @brief Tensão de referência do ADC do Arduino
#define VREF_ADC 5.0

/// @brief Resolução do ADC em bits (Arduino Uno/Nano = 10 bits)
#define ADC_RESOLUTION 10

/// @brief Valor máximo que o ADC pode retornar (2^10 - 1 = 1023)
#define ADC_MAX_VALUE 1023

// ============================================================================
// PARÂMETROS DE FILTRAGEM DO SINAL
// ============================================================================

/**
 * @brief Número de amostras para filtragem estatística
 * 
 * @details Coleta múltiplas amostras, remove outliers e calcula média
 * para reduzir ruído da leitura analógica.
 */
#define NUM_SAMPLES 30

/**
 * @brief Coeficiente do filtro exponencial (EMA - Exponential Moving Average)
 * 
 * @details Controla quanto peso dar à nova leitura vs. histórico.
 * - Valor próximo de 1.0: resposta rápida, menos filtragem
 * - Valor próximo de 0.0: resposta lenta, mais filtragem
 * 
 * @note Fórmula: filtrado = α × novo + (1-α) × filtrado_anterior
 */
#define FILTRO_ALFA 0.1

// ============================================================================
// LIMITES DE PWM (VALORES DE OCR1A)
// ============================================================================

/**
 * @brief Valor de OCR1A com Peltier desligado
 * 
 * @details Com ICR1=1599, OCR_OFF=160 resulta em duty cycle de ~10%,
 * o que mantém o Peltier praticamente desligado.
 * 
 * @note Duty = (OCR1A / (ICR1+1)) × 100%
 */
const uint16_t OCR_OFF = 160;

/**
 * @brief Valor mínimo de OCR1A (máxima corrente no Peltier)
 * 
 * @details OCR_MIN=10 resulta em duty cycle de ~0.6%, que é a máxima
 * potência de resfriamento da célula Peltier.
 * 
 * @warning Valores muito baixos podem danificar o Peltier por excesso de corrente
 */
const uint16_t OCR_MIN = 10;

/**
 * @brief Range de controle do PWM
 * 
 * @details Diferença entre OCR_OFF e OCR_MIN (160 - 10 = 150).
 * Define a faixa de valores utilizáveis para controle.
 */
const uint16_t OCR_RANGE = 150;

// ============================================================================
// VARIÁVEIS GLOBAIS DE ESTADO
// ============================================================================

/// @brief Tensão filtrada atual do sensor (resultado do filtro exponencial)
float tensao_filtrada = 0.0;

/// @brief Flag indicando se é a primeira leitura (para inicializar filtro)
bool primeiro_loop = true;

// Variáveis do algoritmo PID
float erro_anterior = 0.0;           ///< Erro da iteração anterior (para termo derivativo)
float integral_acumulado = 0.0;      ///< Acumulador do termo integral
unsigned long tempo_anterior = 0;    ///< Timestamp da última execução do PID
unsigned long tempo_inicio = 0;      ///< Timestamp do início da execução

// Variáveis para logging e controle de tempo
unsigned long ultimaAtualizacao = 0; ///< Timestamp da última atualização do controle
unsigned long contador_leituras = 0; ///< Contador de iterações do PID

// ============================================================================
// INICIALIZAÇÃO DO SISTEMA
// ============================================================================

/**
 * @brief Função de setup do Arduino - executa uma vez na inicialização
 * 
 * @details Configura:
 * - Comunicação serial para logging
 * - Pino PWM como saída
 * - Timer1 para PWM de 10kHz
 * - Aquecimento do ADC (leituras iniciais para estabilizar)
 * - Inicialização de variáveis de tempo
 * 
 * @section pwm_config Configuração do PWM
 * - Modo: Fast PWM com ICR1 como TOP
 * - Frequência: 16MHz / (prescaler=1 × (ICR1+1)=1600) = 10kHz
 * - Resolução: 1600 níveis (0 a 1599)
 * - Saída: OC1A (pino 9) com modo não-invertido
 */
void setup() {
  // Inicia comunicação serial para monitoramento
  Serial.begin(9600);
  
  // Configura pino PWM como saída
  pinMode(PIN_PWM_TPS, OUTPUT);
  
  // ===== CONFIGURAÇÃO DO TIMER1 PARA PWM 10kHz =====
  // TCCR1A: Timer/Counter Control Register A
  //   COM1A1 = 1: Clear OC1A on Compare Match (modo não-invertido)
  //   WGM11 = 1: Fast PWM com ICR1 como TOP (parte 1/2)
  TCCR1A = (1 << COM1A1) | (1 << WGM11);
  
  // TCCR1B: Timer/Counter Control Register B
  //   WGM13:12 = 11: Fast PWM com ICR1 como TOP (parte 2/2)
  //   CS10 = 1: Prescaler = 1 (sem divisão de clock)
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10);
  
  // ICR1: Input Capture Register (define o TOP do contador)
  //   Freq = F_CPU / (prescaler × (ICR1+1))
  //   10kHz = 16MHz / (1 × 1600) → ICR1 = 1599
  ICR1 = 1599;
  
  // OCR1A: Output Compare Register A (define o duty cycle)
  //   Inicia com Peltier desligado
  OCR1A = OCR_OFF;

  // ===== AQUECIMENTO DO ADC =====
  // Primeiras leituras do ADC são instáveis, descartamos 50 leituras
  Serial.println(F("Inicializando ADC..."));
  for (int i = 0; i < 50; i++) {
    analogRead(pinVTEMP);
    delay(10);
  }
  
  // Inicializa o filtro com a primeira leitura válida
  tensao_filtrada = lerVTEMP();

  // Inicializa contadores de tempo com timestamp atual
  tempo_inicio = millis();
  tempo_anterior = tempo_inicio;
  ultimaAtualizacao = tempo_inicio;
}

// ============================================================================
// LOOP PRINCIPAL
// ============================================================================

/**
 * @brief Loop principal do Arduino - executa continuamente
 * 
 * @details Implementa o ciclo de controle PID:
 * 1. Verifica se é hora de atualizar (baseado em INTERVALO_CONTROLE)
 * 2. Lê temperatura do sensor
 * 3. Calcula erro e termos PID
 * 4. Converte controle para valor de PWM
 * 5. Aplica PWM ao Peltier
 * 6. Exibe informações no monitor serial
 * 
 * @note A lógica de controle só executa a cada INTERVALO_CONTROLE ms
 * @note Entre execuções, há um delay de 10ms para não sobrecarregar o CPU
 */
void loop() {
  unsigned long tempo_atual = millis();
  
  // Verifica se é hora de executar o ciclo de controle
  if (tempo_atual - ultimaAtualizacao >= INTERVALO_CONTROLE) {
    ultimaAtualizacao = tempo_atual;
    
    // ===== ETAPA 1: AQUISIÇÃO DE TEMPERATURA =====
    float vtemp = lerVTEMP();
    float temperatura_atual = COEF_ANGULAR * (vtemp - VTEMP_REF) + TEMP_REF;
    
    // Converte tensão para milivolts para exibição
    float vtemp_mV = vtemp * 1000.0;
    
    // ===== ETAPA 2: CÁLCULO DO INTERVALO DE TEMPO =====
    // dt = tempo decorrido desde última execução (em segundos)
    float dt = (tempo_atual - tempo_anterior) / 1000.0;
    tempo_anterior = tempo_atual;
    
    // ===== ETAPA 3: CÁLCULO DO CONTROLE PID =====
    float termo_p, termo_i, termo_d, controle_calculado;
    float erro_atual;
    
    calcularPID(temperatura_atual, dt, 
                termo_p, termo_i, termo_d, 
                controle_calculado, erro_atual);
    
    // ===== ETAPA 4: CONVERSÃO PARA PWM =====
    uint16_t ocr = converterControleParaOCR(controle_calculado);
    
    // ===== ETAPA 5: APLICAÇÃO DO CONTROLE =====
    OCR1A = ocr;
    
    // ===== ETAPA 6: CÁLCULO DE MÉTRICAS PARA LOGGING =====
    float duty_percent = (ocr * 100.0) / (ICR1 + 1);
    
    // ===== ETAPA 7: DETERMINAÇÃO DE STATUS =====
    String status = determinarStatus(erro_atual, ocr);
    
    // ===== ETAPA 8: LOGGING NO MONITOR SERIAL =====
    float tempo_decorrido = (tempo_atual - tempo_inicio) / 1000.0;
    
    // Formato de saída: [Tempo] VTEMP | Temp | Erro | Peltier | Status
    Serial.print(F("["));
    Serial.print(tempo_decorrido, 1);
    Serial.print(F("s] "));
    
    Serial.print(F("VTEMP: "));
    Serial.print(vtemp, 3);
    Serial.print(F("V | "));
    
    Serial.print(F("Temp: "));
    Serial.print(temperatura_atual, 2);
    Serial.print(F("°C | "));
    
    Serial.print(F("Erro: "));
    if (erro_atual >= 0) Serial.print(F("+"));
    Serial.print(erro_atual, 2);
    Serial.print(F("°C | "));
    
    Serial.print(F("Peltier: "));
    Serial.print(100.0 - duty_percent, 1);  // Inverte para ser intuitivo
    Serial.print(F("% | "));
    
    // Exibe termos PID apenas se relevantes
    if (abs(termo_p) > 1.0 || abs(termo_i) > 1.0 || abs(termo_d) > 1.0) {
      Serial.print(F("PID(P:"));
      Serial.print(termo_p, 1);
      Serial.print(F(" I:"));
      Serial.print(termo_i, 1);
      Serial.print(F(" D:"));
      Serial.print(termo_d, 1);
      Serial.print(F(") | "));
    }
    
    Serial.println(status);
    
    contador_leituras++;
  }
  
  // Pequeno delay para não sobrecarregar o processador
  delay(10);
}

// ============================================================================
// IMPLEMENTAÇÃO DO CONTROLADOR PID
// ============================================================================

/**
 * @brief Calcula os termos PID e o sinal de controle
 * 
 * @details Implementa a equação do PID:
 *          u(t) = Kp×e(t) + Ki×∫e(τ)dτ + Kd×de(t)/dt
 * 
 * Onde:
 * - e(t) = erro atual (temperatura_atual - TEMP_ALVO)
 * - ∫e(τ)dτ = integral do erro ao longo do tempo
 * - de(t)/dt = taxa de variação do erro
 * 
 * @param temperatura_atual Temperatura medida pelo sensor (°C)
 * @param dt Intervalo de tempo desde última execução (segundos)
 * @param[out] termo_p Contribuição do termo proporcional
 * @param[out] termo_i Contribuição do termo integral
 * @param[out] termo_d Contribuição do termo derivativo
 * @param[out] controle_calculado Sinal de controle final (soma dos termos)
 * @param[out] erro_atual Erro calculado (temperatura_atual - TEMP_ALVO)
 * 
 * @note Com INVERTER_POLARIDADE = true:
 *       - Erro positivo (quente demais) → controle negativo → OCR diminui → resfria
 *       - Erro negativo (frio demais) → controle positivo → OCR aumenta → desliga
 * 
 * @section exemplo_calculo Exemplo de Cálculo
 * Cenário: Temp = 27°C, Alvo = 25°C, INVERTER_POLARIDADE = true
 * 1. erro = 27 - 25 = +2°C
 * 2. termo_p = 50 × 2 = +100
 * 3. termo_i = 2 × integral_acumulado (acumula ao longo do tempo)
 * 4. termo_d = 15 × (erro_atual - erro_anterior) / dt
 * 5. controle = termo_p + termo_i + termo_d
 * 6. Com inversão: controle = -controle → valor negativo
 * 7. Controle negativo → OCR diminui → Peltier LIGA e resfria ✅
 */
void calcularPID(float temperatura_atual, float dt,
                 float &termo_p, float &termo_i, float &termo_d,
                 float &controle_calculado, float &erro_atual) {
  
  // ===== PASSO 1: CÁLCULO DO ERRO =====
  // Erro positivo = temperatura acima do alvo (precisa resfriar)
  // Erro negativo = temperatura abaixo do alvo (precisa aquecer/desligar)
  erro_atual = temperatura_atual - TEMP_ALVO;
  
  // ===== PASSO 2: TERMO PROPORCIONAL (P) =====
  // Responde proporcionalmente ao erro atual
  termo_p = Kp * erro_atual;
  
  // ===== PASSO 3: TERMO INTEGRAL (I) =====
  // Acumula o erro ao longo do tempo (integração discreta)
  integral_acumulado += erro_atual * dt;
  
  // ANTI-WINDUP: Limita o valor acumulado para evitar saturação
  if (integral_acumulado > INTEGRAL_LIMITE) {
    integral_acumulado = INTEGRAL_LIMITE;
  } else if (integral_acumulado < -INTEGRAL_LIMITE) {
    integral_acumulado = -INTEGRAL_LIMITE;
  }
  
  termo_i = Ki * integral_acumulado;
  
  // ===== PASSO 4: TERMO DERIVATIVO (D) =====
  // Responde à taxa de variação do erro (diferenciação discreta)
  if (dt > 0 && contador_leituras > 0) {
    float derivada_erro = (erro_atual - erro_anterior) / dt;
    termo_d = Kd * derivada_erro;
  } else {
    termo_d = 0.0;  // Primeira iteração não tem histórico
  }
  
  // Salva erro atual para uso na próxima iteração (cálculo do D)
  erro_anterior = erro_atual;
  
  // ===== PASSO 5: COMBINAÇÃO DOS TERMOS =====
  controle_calculado = termo_p + termo_i + termo_d;
  
  // ===== PASSO 6: INVERSÃO DE POLARIDADE =====
  // Inverte sinal devido à conexão física invertida do Peltier
  if (INVERTER_POLARIDADE) {
    controle_calculado = -controle_calculado;
    termo_p = -termo_p;
    termo_i = -termo_i;
    termo_d = -termo_d;
  }
}

// ============================================================================
// CONVERSÃO DE CONTROLE PARA PWM
// ============================================================================

/**
 * @brief Converte o sinal de controle PID em valor de OCR1A
 * 
 * @details Mapeia o sinal de controle (que pode variar de -∞ a +∞) para
 * o range válido de OCR1A (OCR_MIN a OCR_OFF).
 * 
 * @param controle Sinal de controle calculado pelo PID
 * @return uint16_t Valor de OCR1A a ser aplicado ao Timer1
 * 
 * @section mapeamento Lógica de Mapeamento
 * Com INVERTER_POLARIDADE = true:
 * - controle = 0 (no alvo) → OCR = OCR_OFF (160) → Peltier OFF
 * - controle > 0 (temp alta) → OCR diminui em direção a OCR_MIN → Peltier RESFRIA
 * - controle < 0 (temp baixa) → OCR = OCR_OFF → Peltier permanece OFF
 * 
 * @note A função garante que OCR1A sempre fique entre OCR_MIN e OCR_OFF
 * @note Valores de controle muito grandes são saturados (constrain)
 * 
 * @section tabela Tabela de Conversão
 * | Controle | Descrição           | OCR   | Ação              |
 * |----------|---------------------|-------|-------------------|
 * | 0        | No alvo             | 160   | Peltier OFF       |
 * | +50      | Precisa resfriar    | 110   | Resfriamento leve |
 * | +100     | Muito quente        | 60    | Resfriamento médio|
 * | +150     | Extremamente quente | 10    | Resfriamento MAX  |
 * | -50      | Frio (sem ação)     | 160   | Peltier OFF       |
 */
uint16_t converterControleParaOCR(float controle) {
  uint16_t ocr;
  
  if (controle > 0) {
    // MODO RESFRIAMENTO: controle positivo indica necessidade de resfriar
    // Quanto maior o controle, menor o OCR (mais corrente no Peltier)
    float controle_limitado = constrain(controle, 0, OCR_RANGE);
    ocr = OCR_OFF - (uint16_t)controle_limitado;
  } else {
    // MODO DESLIGADO: controle negativo ou zero mantém Peltier desligado
    // Não há modo de aquecimento ativo neste sistema
    ocr = OCR_OFF;
  }
  
  // Garante que OCR1A fique dentro dos limites de segurança
  ocr = constrain(ocr, OCR_MIN, OCR_OFF);
  
  return ocr;
}

// ============================================================================
// DETERMINAÇÃO DE STATUS DO SISTEMA
// ============================================================================

/**
 * @brief Determina o status descritivo do sistema baseado no erro e OCR
 * 
 * @param erro Erro de temperatura (temperatura_atual - TEMP_ALVO)
 * @param ocr Valor atual de OCR1A aplicado ao PWM
 * @return String Status descritivo do sistema
 * 
 * @section status_table Tabela de Status
 * | Status       | Condição                    | Descrição                    |
 * |--------------|-----------------------------|------------------------------|
 * | NO_ALVO      | |erro| < 0.1°C              | Temperatura estabilizada     |
 * | OFF          | erro < 0.3°C                | Peltier desligado            |
 * | COOL_FRACO   | erro > 0.3°C, OCR > 120     | Resfriamento leve            |
 * | COOL_MEDIO   | erro > 0.3°C, 80<OCR<=120   | Resfriamento moderado        |
 * | COOL_FORTE   | erro > 0.3°C, 40<OCR<=80    | Resfriamento intenso         |
 * | COOL_MAXIMO  | erro > 0.3°C, OCR <= 40     | Resfriamento máximo          |
 * 
 * @note Útil para debug e monitoramento visual do comportamento do sistema
 */
String determinarStatus(float erro, uint16_t ocr) {
  if (abs(erro) < 0.1) {
    return "NO_ALVO";
  } else if (erro > 0.3) {
    // Sistema precisa resfriar - classifica intensidade baseado no OCR
    if (ocr > 120) {
      return "COOL_FRACO";
    } else if (ocr > 80) {
      return "COOL_MEDIO";
    } else if (ocr > 40) {
      return "COOL_FORTE";
    } else {
      return "COOL_MAXIMO";
    }
  } else {
    return "OFF";
  }
}

// ============================================================================
// LEITURA E FILTRAGEM DO SENSOR
// ============================================================================

/**
 * @brief Lê e filtra o sinal de tensão do sensor PICO640
 * 
 * @details Implementa um pipeline de filtragem em múltiplos estágios:
 * 1. Amostragem múltipla (NUM_SAMPLES leituras)
 * 2. Ordenação das amostras
 * 3. Remoção de outliers (descarta 3 maiores e 3 menores)
 * 4. Média das amostras centrais
 * 5. Conversão ADC → Tensão
 * 6. Filtro exponencial (EMA)
 * 
 * @return float Tensão filtrada em Volts
 * 
 * @section filtro_estatistico Filtragem Estatística
 * - Coleta 30 amostras
 * - Remove os 6 outliers (3 min + 3 max)
 * - Calcula média dos 24 valores centrais
 * - Reduz ruído de alta frequência
 * 
 * @section filtro_exponencial Filtro Exponencial (EMA)
 * Fórmula: y[n] = α×x[n] + (1-α)×y[n-1]
 * - α = FILTRO_ALFA = 0.1
 * - Suaviza variações bruscas
 * - Mantém 90% do histórico + 10% da nova leitura
 * 
 * @note Na primeira execução, inicializa o filtro sem histórico
 * @note Delay de 200µs entre amostras para estabilizar o ADC
 * 
 * @section conversao Conversão ADC → Tensão
 * Tensão = (valor_ADC × VREF_ADC) / ADC_MAX_VALUE
 * Tensão = (valor_ADC × 5.0V) / 1023
 */
float lerVTEMP() {
  int amostras[NUM_SAMPLES];
  
  // ===== ETAPA 1: COLETA DE AMOSTRAS =====
  // Coleta múltiplas leituras do ADC com pequeno delay entre elas
  for (int i = 0; i < NUM_SAMPLES; i++) {
    amostras[i] = analogRead(pinVTEMP);
    delayMicroseconds(200);  // Aguarda estabilização do ADC
  }
  
  // ===== ETAPA 2: ORDENAÇÃO (BUBBLE SORT) =====
  // Ordena o array para permitir remoção de outliers
  for (int i = 0; i < NUM_SAMPLES - 1; i++) {
    for (int j = 0; j < NUM_SAMPLES - i - 1; j++) {
      if (amostras[j] > amostras[j + 1]) {
        // Troca elementos adjacentes se estiverem fora de ordem
        int temp = amostras[j];
        amostras[j] = amostras[j + 1];
        amostras[j + 1] = temp;
      }
    }
  }
  
  // ===== ETAPA 3: REMOÇÃO DE OUTLIERS E MÉDIA =====
  // Remove os 3 menores e 3 maiores valores (total: 6 outliers)
  // Calcula média dos 24 valores centrais (índices 3 a 26)
  unsigned long soma = 0;
  for (int i = 3; i < NUM_SAMPLES - 3; i++) {
    soma += amostras[i];
  }
  
  // Média das amostras filtradas
  float adcMedio = soma / 24.0;
  
  // ===== ETAPA 4: CONVERSÃO ADC → TENSÃO =====
  // Converte valor ADC (0-1023) para tensão (0-5V)
  float tensao_raw = (adcMedio * VREF_ADC) / ADC_MAX_VALUE;
  
  // ===== ETAPA 5: FILTRO EXPONENCIAL (EMA) =====
  if (primeiro_loop) {
    // Primeira execução: inicializa filtro sem histórico
    tensao_filtrada = tensao_raw;
    primeiro_loop = false;
  } else {
    // Aplica filtro exponencial: 10% novo valor + 90% histórico
    tensao_filtrada = (FILTRO_ALFA * tensao_raw) + ((1 - FILTRO_ALFA) * tensao_filtrada);
  }
  
  return tensao_filtrada;
}