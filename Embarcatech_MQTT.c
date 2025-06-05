#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/uart.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "lib/ssd1306.h"
#include "lib/ws2812.h"
#include "lib/font.h"
#include "lib/mqtt_client.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "pico/bootrom.h"
#include <math.h>
#include "semphr.h"

//configuração do i2c para o display
#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define address 0x3C

//pinos do adc para o joystick
#define ADC_JOYSTICK_X 26
#define ADC_JOYSTICK_Y 27
#define DEADZONE 100

//pinos dos led RGB
#define LED_RED 13
#define LED_BLUE 12
#define LED_GREEN  11

#define buttonB 6

#define BUZZER_PIN 10   //pino do buzzer

uint64_t last_time = 0;

bool color = true;  //variavel que indica que se o pixel está ligado ou desligado
ssd1306_t ssd; //inicializa a estrutura do display

//definção da estrutura que armazena os dados do joystick
typedef struct
{
    uint16_t x_position; //nivel da agua
    uint16_t x_center;
    float x_percentual;

    uint16_t y_position; //volume de chuva
    uint16_t y_center;
    float y_percentual;

} joystick_data_t;

// Filas para comunicação entre tasks
QueueHandle_t xQueueJoystickData;
QueueHandle_t xQueueJoystickForMQTT;  // Fila dedicada para MQTT

MQTT_CLIENT_DATA_T *global_mqtt_state = NULL;

// Flags de controle MQTT
volatile bool buzzer_enabled = true;    // Flag para habilitar/desabilitar buzzer
volatile bool leds_enabled = true;      // Flag para habilitar/desabilitar LEDs
volatile bool alerts_enabled = true;    // Flag para habilitar/desabilitar alertas gerais
volatile bool matrix_enabled = true;    // Flag para habilitar/desabilitar Matriz de leds

// Mutex para proteger acesso às flags
SemaphoreHandle_t xFlagsMutex;


void vJoystickTask(void *params);
void vLedTask(void *params);
void vMatrixTask(void *params);
void vDisplayTask(void *params);
void vBuzzerTask(void *params);
void vMQTTTask(void *params);
void play_buzzer(uint freq, float duty_cycle);
bool deadZoneCheck(uint16_t joy_position, uint16_t joy_center);
void gpio_irq_handler(uint gpio, uint32_t events);
void vMQTTPublishTask(void *params);
void set_buzzer_flag(bool enabled);
void set_leds_flag(bool enabled);
void set_alerts_flag(bool enabled);
void set_matrix_flag(bool enabled);

void set_global_mqtt_state(MQTT_CLIENT_DATA_T *state) {
    global_mqtt_state = state;
}

int main()
{
    stdio_init_all();

    gpio_init(buttonB);
    gpio_set_dir(buttonB, GPIO_IN);
    gpio_pull_up(buttonB);

    gpio_set_irq_enabled_with_callback(buttonB, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

    // Criar mutex para as flags
    xFlagsMutex = xSemaphoreCreateMutex();
    if (xFlagsMutex == NULL) {
        panic("Mutex creation failed");
    }

    // CORREÇÃO: Criar filas menores para evitar acúmulo de dados antigos
    xQueueJoystickData = xQueueCreate(2, sizeof(joystick_data_t));
    xQueueJoystickForMQTT = xQueueCreate(3, sizeof(joystick_data_t));

    if (xQueueJoystickData == NULL || xQueueJoystickForMQTT == NULL) {
        printf("Erro ao criar filas!\n");
        panic("Queue creation failed");
    }

    // CORREÇÃO: Incluir a task de LED que estava faltando
    xTaskCreate(vJoystickTask, "Joystick Task", 512, NULL, 4, NULL);
    xTaskCreate(vLedTask, "LED Task", 256, NULL, 2, NULL);
    xTaskCreate(vMatrixTask, "Matrix Task", 512, NULL, 2, NULL);
    xTaskCreate(vDisplayTask, "Display Task", 512, NULL, 2, NULL);
    xTaskCreate(vBuzzerTask, "Buzzer Task", 256, NULL, 2, NULL);
    xTaskCreate(vMQTTTask, "MQTT Task", 1024, NULL, 3, NULL);
    xTaskCreate(vMQTTPublishTask, "MQTT Publish Task", 512, NULL, 3, NULL);

    vTaskStartScheduler();
    panic_unsupported();
}

void vMQTTTask(void *params){
    INFO_printf("mqtt client starting\n");

    // Cria registro com os dados do cliente
    MQTT_CLIENT_DATA_T state;
    
    // IMPORTANTE: Definir o ponteiro global ANTES de qualquer tentativa de conexão
    set_global_mqtt_state(&state);

    // Inicializar todos os campos da estrutura
    memset(&state, 0, sizeof(MQTT_CLIENT_DATA_T));
    state.connect_done = false;
    state.subscribe_count = 0;
    state.stop_client = false;

    // Inicializa a arquitetura do cyw43
    if (cyw43_arch_init()) {
        panic("Failed to inizialize CYW43");
    }

    // Usa identificador único da placa
    char unique_id_buf[5];
    pico_get_unique_board_id_string(unique_id_buf, sizeof(unique_id_buf));
    for(int i=0; i < sizeof(unique_id_buf) - 1; i++) {
        unique_id_buf[i] = tolower(unique_id_buf[i]);
    }

    // Gera nome único, Ex: pico1234
    char client_id_buf[sizeof(MQTT_DEVICE_NAME) + sizeof(unique_id_buf) - 1];
    memcpy(&client_id_buf[0], MQTT_DEVICE_NAME, sizeof(MQTT_DEVICE_NAME) - 1);
    memcpy(&client_id_buf[sizeof(MQTT_DEVICE_NAME) - 1], unique_id_buf, sizeof(unique_id_buf) - 1);
    client_id_buf[sizeof(client_id_buf) - 1] = 0;
    INFO_printf("Device name %s\n", client_id_buf);

    state.mqtt_client_info.client_id = client_id_buf;
    state.mqtt_client_info.keep_alive = MQTT_KEEP_ALIVE_S;

    #if defined(MQTT_USERNAME) && defined(MQTT_PASSWORD)
        state.mqtt_client_info.client_user = MQTT_USERNAME;
        state.mqtt_client_info.client_pass = MQTT_PASSWORD;
    #else
        state.mqtt_client_info.client_user = NULL;
        state.mqtt_client_info.client_pass = NULL;
    #endif

    static char will_topic[MQTT_TOPIC_LEN];
    strncpy(will_topic, full_topic(&state, MQTT_WILL_TOPIC), sizeof(will_topic));
    state.mqtt_client_info.will_topic = will_topic;
    state.mqtt_client_info.will_msg = MQTT_WILL_MSG;
    state.mqtt_client_info.will_qos = MQTT_WILL_QOS;
    state.mqtt_client_info.will_retain = true;

    // Conectar à rede WiFI
    cyw43_arch_enable_sta_mode();
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        panic("Failed to connect");
    }
    INFO_printf("\nConnected to Wifi\n");

    // Fazer um pedido de DNS para o endereço IP do servidor MQTT
    cyw43_arch_lwip_begin();
    int err = dns_gethostbyname(MQTT_SERVER, &state.mqtt_server_address, dns_found, &state);
    cyw43_arch_lwip_end();

    // Se tiver o endereço, inicia o cliente
    if (err == ERR_OK) {
        start_client(&state);
    } else if (err != ERR_INPROGRESS) {
        panic("dns request failed");
    }

    // CORREÇÃO: Aguardar a primeira conexão ser estabelecida
    while (!state.connect_done) {
        cyw43_arch_poll();
        cyw43_arch_wait_for_work_until(make_timeout_time_ms(100));
    }

    INFO_printf("Primeira conexão MQTT estabelecida com sucesso\n");

    // Loop principal - monitoramento de conexão
    while (1) {
        cyw43_arch_poll();
        
        // Verificar se está conectado
        if (mqtt_client_is_connected(state.mqtt_client_inst)) {
            // Conectado - aguardar um pouco
            cyw43_arch_wait_for_work_until(make_timeout_time_ms(1000));
        } else {
            // Desconectado - tentar reconectar
            INFO_printf("Conexão MQTT perdida. Tentando reconectar...\n");
            state.connect_done = false;
            
            // Aguardar um pouco antes de tentar reconectar
            vTaskDelay(pdMS_TO_TICKS(5000));
            
            reconnect_mqtt(&state);
            
            // Aguardar reconexão
            int timeout_count = 0;
            while (!state.connect_done && timeout_count < 30) { // 30 segundos timeout
                cyw43_arch_poll();
                cyw43_arch_wait_for_work_until(make_timeout_time_ms(1000));
                timeout_count++;
            }
            
            if (!state.connect_done) {
                ERROR_printf("Falha na reconexão MQTT após 30 segundos\n");
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    INFO_printf("mqtt client exiting\n");
}

void vJoystickTask(void *params)
{
    adc_init();
    adc_gpio_init(ADC_JOYSTICK_Y);
    adc_gpio_init(ADC_JOYSTICK_X);

    joystick_data_t joydata;

    //bloco com leitura inicial do adc para definir onde é o centro do joystick
    sleep_ms(500);  // Aumentar delay inicial para estabilização

    adc_select_input(0);
    joydata.y_center = adc_read();
   
    adc_select_input(1);
    joydata.x_center = adc_read();

    printf("\nFirst Values x_center, y_center: %d, %d\n", joydata.x_center, joydata.y_center);

    sleep_ms(100);
    //fim da leitura inicial

    while (true)
    {
        adc_select_input(0); // GPIO 26 = ADC0
        joydata.y_position = adc_read();

        adc_select_input(1); // GPIO 27 = ADC1
        joydata.x_position = adc_read();

        //verifica se o eixo X está dentro da zona morta
        if(deadZoneCheck(joydata.x_position, joydata.x_center))
        {
            joydata.x_position = joydata.x_center;
        }

        //verifica se o eixo Y está dentro da zona morta
        if(deadZoneCheck(joydata.y_position, joydata.y_center))
        {
            joydata.y_position = joydata.y_center;
        }

        // Cálculo corrigido dos percentuais
        joydata.x_percentual = (fabs((float)joydata.x_position - joydata.x_center) / (joydata.x_center > 0 ? joydata.x_center : 1)) * 100;
        joydata.y_percentual = (fabs((float)joydata.y_position - joydata.y_center) / (joydata.y_center > 0 ? joydata.y_center : 1)) * 100;

        // Limitar percentuais a 100%
        if (joydata.x_percentual > 100.0f) joydata.x_percentual = 100.0f;
        if (joydata.y_percentual > 100.0f) joydata.y_percentual = 100.0f;

        
        // CORREÇÃO: Limpar filas antes de enviar novos dados para evitar acúmulo
        joystick_data_t temp_data;
        while (xQueueReceive(xQueueJoystickData, &temp_data, 0) == pdTRUE) {
            // Limpar fila principal
        }
        while (xQueueReceive(xQueueJoystickForMQTT, &temp_data, 0) == pdTRUE) {
            // Limpar fila MQTT
        }

        // Enviar dados mais recentes para ambas as filas
        if (xQueueSend(xQueueJoystickData, &joydata, 0) != pdTRUE) {
            printf("ERRO: Falha ao enviar para xQueueJoystickData\n");
        }
        if (xQueueSend(xQueueJoystickForMQTT, &joydata, 0) != pdTRUE) {
            printf("ERRO: Falha ao enviar para xQueueJoystickForMQTT\n");
        }
        
        vTaskDelay(pdMS_TO_TICKS(200));  // Reduzir frequência para 5Hz
    }
}

void vLedTask(void *params)
{
    gpio_init(LED_RED);
    gpio_init(LED_GREEN);

    gpio_set_dir(LED_RED, GPIO_OUT);
    gpio_set_dir(LED_GREEN, GPIO_OUT);
    

    joystick_data_t joydata;

    while (true)
    {
        if (xQueueReceive(xQueueJoystickData, &joydata, portMAX_DELAY) == pdTRUE)
        {
    
            //led vermelho se o nível da água for maior que 70% ou volume de chuva maior que 80%
            if(joydata.x_percentual > 70.0 || joydata.y_percentual > 80.0)
            {
                gpio_put(LED_RED, true);
                gpio_put(LED_GREEN, false);
                

            }//led amarelo se o nível da água for maior que 40% ou volume de chuva maior que 50%
            else if(joydata.x_percentual > 40.0 || joydata.y_percentual > 50.0) 
            {
                gpio_put(LED_RED, true);
                gpio_put(LED_GREEN, true);

            }else{
                //Desliga os led se o nível da água e volume de chuva estiverem normais
                gpio_put(LED_RED, false);
                gpio_put(LED_GREEN, false);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // Atualiza a cada 50ms
    }
}

void vMatrixTask(void *params)
{
    npInit(LED_PIN);        //inicializa matriz de led
    npClear();              //limpa a matriz

    //frame de alerta
    int frameA[5][5] = {

        {0, 0, 0, 0, 0},
        {0, 1, 1, 1, 0},
        {0, 1, 1, 1, 0},
        {0, 1, 1, 1, 0},
        {0, 0, 0, 0, 0}
        };

    //fram de perigo
    int frameD[5][5] = {
        {1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1}
        };
            
    joystick_data_t joydata;


    while(true)
    {
        if (xQueueReceive(xQueueJoystickData, &joydata, portMAX_DELAY) == pdTRUE)
        {
            //led amarelo se o nível da água for maior que 70% ou volume de chuva maior que 80%
            if(joydata.x_percentual > 70.0 || joydata.y_percentual > 80.0)
            {
                print_frame(frameD, 80, 0,0);
                
            }//led amarelo se o nível da água for maior que 40% ou volume de chuva maior que 50%
            else if(joydata.x_percentual > 40.0 || joydata.y_percentual > 50.0) 
            {
                print_frame(frameA, 50, 50, 0);

            }else{
                //Desliga os led se o nível da água e volume de chuva estiverem normais
                npClear();
            }
        }

    }
}

void vDisplayTask(void *params)
{
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    ssd1306_t ssd;
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, address, I2C_PORT);
    ssd1306_config(&ssd);
    ssd1306_send_data(&ssd);
    //limpa o display. O display inicia com todos os pixels apagados.
    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);

    joystick_data_t joydata;
    char x_str[10];
    char y_str[10];

    while(true)
    {
        if (xQueueReceive(xQueueJoystickData, &joydata, portMAX_DELAY) == pdTRUE)
        {
            sprintf(x_str, "%.2f%%", joydata.x_percentual);
            sprintf(y_str, "%.2f%%", joydata.y_percentual);

            if(joydata.x_percentual > 70.0 || joydata.y_percentual > 80.0)
            {
                ssd1306_fill(&ssd, !color);  // Limpa o display
                ssd1306_rect(&ssd, 3, 3, 122, 60, color, !color); // Desenha um retângulo
                ssd1306_hline(&ssd, 3, 122, 32, color); //desenha a linha horizontal que divide o display no meio
                ssd1306_vline(&ssd, 61, 32, 60, color); //desenha a linha vertical que divide o display no meio
                ssd1306_draw_string(&ssd, "PERIGO!", 37, 8);
                ssd1306_draw_string(&ssd, "NIVEL ALTO!", 18, 20);
                ssd1306_draw_string(&ssd, "N.AGUA", 8, 34);
                ssd1306_draw_string(&ssd, x_str, 8, 46);
                ssd1306_draw_string(&ssd, "V.CHUVA", 65, 34);
                ssd1306_draw_string(&ssd, y_str, 67, 46);
                ssd1306_send_data(&ssd);  // Atualiza o display
            }//led amarelo se o nível da água for maior que 40% ou volume de chuva maior que 50%
            else if(joydata.x_percentual > 40.0 || joydata.y_percentual > 50.0) 
            {
                ssd1306_fill(&ssd, !color);  // Limpa o display
                ssd1306_rect(&ssd, 3, 3, 122, 60, color, !color); // Desenha um retângulo
                ssd1306_hline(&ssd, 3, 122, 32, color);
                ssd1306_vline(&ssd, 61, 32, 60, color);
                ssd1306_draw_string(&ssd, "ALERTA", 37, 8);
                ssd1306_draw_string(&ssd, "NIVEL MEDIO", 16, 20);
                ssd1306_draw_string(&ssd, "N.AGUA", 8, 34);
                ssd1306_draw_string(&ssd, x_str, 8, 46);
                ssd1306_draw_string(&ssd, "V.CHUVA", 65, 34);
                ssd1306_draw_string(&ssd, y_str, 67, 46);
                ssd1306_send_data(&ssd);  // Atualiza o display
            }else{
                ssd1306_fill(&ssd, !color);  // Limpa o display
                ssd1306_rect(&ssd, 3, 3, 122, 60, color, !color); // Desenha um retângulo
                ssd1306_hline(&ssd, 3, 122, 32, color);
                ssd1306_vline(&ssd, 61, 32, 60, color);
                ssd1306_draw_string(&ssd, "NIVEL SEGURO", 15, 15);
                ssd1306_draw_string(&ssd, "N.AGUA", 8, 34);
                ssd1306_draw_string(&ssd, x_str, 8, 46);
                ssd1306_draw_string(&ssd, "V.CHUVA", 65, 34);
                ssd1306_draw_string(&ssd, y_str, 67, 46);
                ssd1306_send_data(&ssd);  // Atualiza o display
            }
        }
    }
}

void vBuzzerTask(void *params)
{
    uint wrap_buzzer = 10000;
    uint buzzer_freq = 1000;

    gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);

    uint slice_num = pwm_gpio_to_slice_num(BUZZER_PIN);
    pwm_set_wrap(slice_num, wrap_buzzer);
    pwm_set_enabled(slice_num, true);

    joystick_data_t joydata;
    bool local_buzzer_enabled;
    bool local_alerts_enabled;

    while(1)
    {
        if (xQueueReceive(xQueueJoystickData, &joydata, portMAX_DELAY) == pdTRUE)
        {
            // Ler flags de forma thread-safe
            if (xSemaphoreTake(xFlagsMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                local_buzzer_enabled = buzzer_enabled;
                local_alerts_enabled = alerts_enabled;
                xSemaphoreGive(xFlagsMutex);
            } else {
                // Se não conseguir o mutex, usar valores padrão
                local_buzzer_enabled = true;
                local_alerts_enabled = true;
            }

            // Só tocar buzzer se estiver habilitado
            if (local_buzzer_enabled && local_alerts_enabled) {
                if(joydata.x_percentual > 70.0 || joydata.y_percentual > 80.0)
                {
                    play_buzzer(buzzer_freq, 0.5); // 1KHz, 50% duty cycle
                    vTaskDelay(pdMS_TO_TICKS(300));
                    play_buzzer(buzzer_freq, 0); // Desligar
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
                else if(joydata.x_percentual > 40.0 || joydata.y_percentual > 50.0) 
                {
                    play_buzzer(buzzer_freq/2, 0.5); // 500Hz, 50% duty cycle
                    vTaskDelay(pdMS_TO_TICKS(300));
                    play_buzzer(buzzer_freq/2, 0); // Desligar
                    vTaskDelay(pdMS_TO_TICKS(1000));
                }
                else {
                    play_buzzer(buzzer_freq, 0); // Desligar buzzer
                }
            } else {
                // Buzzer desabilitado - manter desligado
                play_buzzer(buzzer_freq, 0);
            }
        }
    }
}

void play_buzzer(uint freq, float duty_cycle) {
    uint slice = pwm_gpio_to_slice_num(BUZZER_PIN); //armazena a slice do PWM do buzzer
    uint clock_divider = 4; // Define o divisor do clock (ajuste se necessário)
    uint wrap = clock_get_hz(clk_sys) / (clock_divider * freq); //calcula o wrap do PWM buzzer

    pwm_set_clkdiv(slice, clock_divider);   //configura o divisor de clock
    pwm_set_wrap(slice, wrap);  //configura o wrap
    pwm_set_gpio_level(BUZZER_PIN, wrap * duty_cycle);  //ativa o pwm
}

void gpio_irq_handler(uint gpio, uint32_t events)
{
    uint64_t current_time = to_ms_since_boot(get_absolute_time()); //armazena o registrado no momento da interrupção
    
    if(current_time - last_time > 200)  //checa se passou pelo menos 200 ms entre duas ativações do botão
    {
        npClear();  //limpa a matriz de leds
        reset_usb_boot(0, 0);   //coloca em modo bootloader
    
        last_time = current_time; //atualiza as variáveis de tempo
    }
}

bool deadZoneCheck(uint16_t joy_position, uint16_t joy_center)
{
    if(joy_position > (joy_center - DEADZONE) && joy_position < (joy_center + DEADZONE))
    {
        return true; //retorna true se a leitura estiver dentro da zona morta
    }

    return false; //retorna false se a leitura estiver fora da zona morta
}

void vMQTTPublishTask(void *params) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPublishFrequency = pdMS_TO_TICKS(2000);  // Publicar a cada 2 segundos
    
    INFO_printf("MQTT Publish Task iniciada\n");
    
    // Aguardar inicialização do MQTT
    while (global_mqtt_state == NULL) {
        INFO_printf("Aguardando inicialização do MQTT...\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    INFO_printf("Estado MQTT disponível, iniciando publicações...\n");
    
    while (true) {
        // Verificar se está conectado
        if (global_mqtt_state != NULL && 
            global_mqtt_state->connect_done && 
            mqtt_client_is_connected(global_mqtt_state->mqtt_client_inst)) {
            
            INFO_printf("=== CICLO DE PUBLICAÇÃO MQTT ===\n");
            
            // CORREÇÃO: Verificar se há dados na fila antes de publicar
            UBaseType_t items_in_queue = uxQueueMessagesWaiting(xQueueJoystickForMQTT);
            INFO_printf("Itens na fila MQTT: %d\n", items_in_queue);
            
            if (items_in_queue > 0) {
                publish_water_level(global_mqtt_state);
            } else {
                INFO_printf("Nenhum dado disponível para publicação\n");
            }
            
        } else {
            DEBUG_printf("MQTT não conectado. Estado: connect_done=%d\n", 
                        global_mqtt_state ? global_mqtt_state->connect_done : -1);
        }
        
        // Aguarda o próximo ciclo
        vTaskDelayUntil(&xLastWakeTime, xPublishFrequency);
    }
}

void set_buzzer_flag(bool enabled) {
    if (xSemaphoreTake(xFlagsMutex, portMAX_DELAY) == pdTRUE) {
        buzzer_enabled = enabled;
        printf("Buzzer %s via MQTT\n", enabled ? "HABILITADO" : "DESABILITADO");
        xSemaphoreGive(xFlagsMutex);
    }
}

void set_leds_flag(bool enabled) {
    if (xSemaphoreTake(xFlagsMutex, portMAX_DELAY) == pdTRUE) {
        leds_enabled = enabled;
        printf("LEDs %s via MQTT\n", enabled ? "HABILITADOS" : "DESABILITADOS");
        
        // CORREÇÃO: Desligar TODOS os LEDs quando desabilitado
        if (!enabled) {
            gpio_put(LED_RED, false);
            gpio_put(LED_GREEN, false);
            gpio_put(LED_BLUE, false); 
        }
        
        xSemaphoreGive(xFlagsMutex);
    }
}

void set_alerts_flag(bool enabled) {
    if (xSemaphoreTake(xFlagsMutex, portMAX_DELAY) == pdTRUE) {
        alerts_enabled = enabled;
        printf("Alertas %s via MQTT\n", enabled ? "HABILITADOS" : "DESABILITADOS");
        xSemaphoreGive(xFlagsMutex);
    }
}

void set_matrix_flag(bool enabled) {
    if (xSemaphoreTake(xFlagsMutex, portMAX_DELAY) == pdTRUE) {
        matrix_enabled = enabled;
        printf("Matriz de LEDs %s via MQTT\n", enabled ? "HABILITADA" : "DESABILITADA");
        
        // Se desabilitado, limpar matriz imediatamente
        if (!enabled) {
            npClear();
        }
        
        xSemaphoreGive(xFlagsMutex);
    }
}