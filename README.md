# Embarcatech_MQTT

Repositório criado para o desenvolvimento de uma estação de monitoramento de enchentes com conectividade IoT utilizando os conceitos de multitarefas, filas no FreeRTOS e comunicação MQTT via Wi-Fi.
Este projeto implementa uma simulação de estação de monitoramento utilizando os seguintes periféricos da placa BitDogLab:

- LED RGB
- Matriz de LEDs WS2812
- Display OLED SSD1306
- Buzzer Passivo
- Joystick (ADC)
- Conectividade Wi-Fi (CYW43)

O software é desenvolvido com o uso das tasks do FreeRTOS, permitindo o gerenciamento paralelo de cada periférico e comunicação IoT via protocolo MQTT. Ao todo, são implementadas sete tasks distintas, 
sendo cinco para controle de periféricos e duas para comunicação MQTT.

# Arquitetura do Sistema

1. **Sistema de Filas**
   
Cada task consome dados de duas filas distintas que são populadas com as leituras obtidas do Joystick através do ADC:

- xQueueJoystickData: Fila para comunicação local entre tarefas de controle de periféricos
- xQueueJoystickForMQTT: Fila dedicada exclusivamente para envio de dados via MQTT

2. **Tasks Implementadas**

- vJoystickTask: Leitura contínua do joystick (eixos X e Y representando nível de água e volume de chuva)
- vLedTask: Controle dos LEDs RGB baseado nos níveis de alerta
- vMatrixTask: Controle da matriz de LEDs WS2812 com padrões visuais
- vDisplayTask: Gerenciamento do display OLED com interface informativa
- vBuzzerTask: Sistema de alerta sonoro com diferentes padrões
- vMQTTTask: Inicialização e manutenção da conexão MQTT/Wi-Fi
- vMQTTPublishTask: Publicação periódica de dados dos sensores

# Funcionamento dos Alertas

1. Nível Normal

- Condição: Nível de água < 40% E volume de chuva < 50%
- Display: Mensagem "NÍVEL SEGURO" com percentuais atualizados
- Buzzer: Silencioso
- Matriz: Completamente apagada
- LED RGB: Desligado

2. Nível Médio (Alerta)

- Condição: Nível de água > 40% OU volume de chuva > 50%
- Display: Mensagem "ALERTA - NÍVEL MÉDIO" com percentuais atualizados
- Buzzer: Bipes lentos de 500Hz (300ms ligado, 1s pausado)
- Matriz: Quadrado amarelo 3x3 centralizado
- LED RGB: Amarelo (vermelho + verde)

3. Nível Alto (Perigo)

- Condição: Nível de água > 70% OU volume de chuva > 80%
- Display: Mensagem "PERIGO! NÍVEL ALTO!" com percentuais atualizados
- Buzzer: Bipes rápidos de 1kHz (300ms ligado, 100ms pausado)
- Matriz: Quadrado vermelho 5x5 completo
- LED RGB: Vermelho

# Conectividade IoT - MQTT

1. **Tópicos de Publicação (a cada 2 segundos)**

- /adc/nivel_agua: Percentual do nível da água
- /adc/volume_chuva: Percentual do volume de chuva

2. **Tópicos de Controle Remoto**

- /control/buzzer: Habilita/desabilita o sistema de alerta sonoro (1 = ligado, 0 = desligado)
- /control/leds: Controla habilitação dos LEDs RGB
- /control/alerts: Controle geral de alertas
- /control/matrix: Controla habilitação da matriz de LEDs
- /led: Controla o LED interno da placa
- /ping: Retorna o tempo de atividade do sistema

# Recursos de Robustez

- Reconexão Automática: Sistema de reconexão MQTT com timeout configurável
- Identificador Único: Utiliza ID único da placa para evitar conflitos na rede
- Proteção Thread-Safe: Mutex para proteger acesso às flags de controle
- Limpeza de Filas: Evita acúmulo de dados obsoletos
- Debounce: Implementado via software para o botão de reset

# Configuração de Hardware

Pinagem:

- Display I2C: SDA (GPIO 14), SCL (GPIO 15)
- Joystick ADC: X (GPIO 26), Y (GPIO 27)
- LEDs RGB: Vermelho (GPIO 13), Verde (GPIO 11), Azul (GPIO 12)
- Buzzer PWM: GPIO 10
- Botão Reset: GPIO 6
- Matriz WS2812: Definido em LED_PIN (verificar lib/ws2812.h)

# Instruções de Compilação

Para compilar o código, são necessárias as seguintes dependências:

- Raspberry Pi Pico SDK
- CMake
- FreeRTOS instalado
- Bibliotecas específicas: ssd1306, ws2812, mqtt_client

# Configuração Wi-Fi e MQTT

Antes da compilação, configure no arquivo de cabeçalho apropriado:

- #define WIFI_SSID "sua_rede_wifi"
- #define WIFI_PASSWORD "sua_senha_wifi"
- #define MQTT_SERVER "endereco_do_broker_mqtt"
- #define MQTT_DEVICE_NAME "pico"

# Testes e Validação

O firmware foi testado utilizando:

- Broker MQTT: Mosquitto
- Cliente MQTT: MQTT Explorer e MQTT Panel
- Monitoramento: Plotagem de gráficos em tempo real
- Controle Remoto: Botões de controle via interface do MQTT Panel
  
# Build

- Importe o projeto através da extensão do Raspberry Pi Pico
- Configure o diretório do FreeRTOS no arquivo CMakeLists.txt
- Configure as credenciais Wi-Fi e servidor MQTT
- Construa (build) o projeto utilizando o CMake

