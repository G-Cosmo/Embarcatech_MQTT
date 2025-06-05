/* AULA IoT - Ricardo Prates - 001 - Cliente MQTT - Publisher:/Temperatura; Subscribed:/led
 *
 * Material de suporte - 27/05/2025
 * 
 * Código adaptado de: https://github.com/raspberrypi/pico-examples/tree/master/pico_w/wifi/mqtt 
 */

#include "mqtt_client.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <math.h>

// Variável global para acesso à fila do joystick (usar a fila dedicada ao MQTT)
extern QueueHandle_t xQueueJoystickForMQTT;

// Requisição para publicar
void pub_request_cb(__unused void *arg, err_t err) {
    if (err != ERR_OK) {
        ERROR_printf("pub_request_cb failed %d\n", err);
    } else {
        INFO_printf("Publicação confirmada pelo broker\n");
    }
}

//Topico MQTT
const char *full_topic(MQTT_CLIENT_DATA_T *state, const char *name) {
#if MQTT_UNIQUE_TOPIC
    snprintf(state->full_topic_buffer, sizeof(state->full_topic_buffer), "/%s%s", state->mqtt_client_info.client_id, name);
    return state->full_topic_buffer;
#else
    return name;
#endif
}

// Controle do LED 
void control_led(MQTT_CLIENT_DATA_T *state, bool on) {
    // Publish state on /state topic and on/off led board
    const char* message = on ? "On" : "Off";
    if (on)
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    else
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);

    mqtt_publish(state->mqtt_client_inst, full_topic(state, "/led/state"), message, strlen(message), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
}

// Publicar dados do ADC (nível da água e volume de chuva)
void publish_water_level(MQTT_CLIENT_DATA_T *state) {
    // Definir a estrutura localmente (deve ser igual à do arquivo principal)
    typedef struct {
        uint16_t x_position;
        uint16_t x_center;
        float x_percentual;
        uint16_t y_position;
        uint16_t y_center;
        float y_percentual;
    } joystick_data_t;
    
    joystick_data_t joydata;

    // CORREÇÃO: Ler o dado mais recente da fila (sem timeout)
    if (xQueueReceive(xQueueJoystickForMQTT, &joydata, 0) == pdTRUE) {
        
        // CORREÇÃO: Limpar qualquer dado antigo restante na fila
        joystick_data_t temp_data;
        while (xQueueReceive(xQueueJoystickForMQTT, &temp_data, 0) == pdTRUE) {
            // Atualizar com o dado mais recente
            joydata = temp_data;
        }
        
        char buf_x[16], buf_y[16];
        snprintf(buf_x, sizeof(buf_x), "%.2f", joydata.x_percentual);
        snprintf(buf_y, sizeof(buf_y), "%.2f", joydata.y_percentual);

        INFO_printf("=== PUBLICAÇÃO MQTT ===\n");
        INFO_printf("Dados ATUAIS da fila: x=%.2f%% (nivel_agua), y=%.2f%% (volume_chuva)\n", 
                   joydata.x_percentual, joydata.y_percentual);
        INFO_printf("Posições ADC: x=%d (centro=%d), y=%d (centro=%d)\n",
                   joydata.x_position, joydata.x_center, 
                   joydata.y_position, joydata.y_center);

        // Verificar se o cliente MQTT está conectado
        if (mqtt_client_is_connected(state->mqtt_client_inst)) {
            INFO_printf("Publicando dados ADC nos tópicos MQTT\n");
            
            err_t err1 = mqtt_publish(state->mqtt_client_inst,
                         full_topic(state, "/adc/nivel_agua"),
                         buf_x, strlen(buf_x),
                         MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN,
                         pub_request_cb, state);

            err_t err2 = mqtt_publish(state->mqtt_client_inst,
                         full_topic(state, "/adc/volume_chuva"),
                         buf_y, strlen(buf_y),
                         MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN,
                         pub_request_cb, state);

            if (err1 == ERR_OK && err2 == ERR_OK) {
                INFO_printf("✓ Publicação MQTT bem-sucedida: nivel_agua=%.2f%% volume_chuva=%.2f%%\n", 
                           joydata.x_percentual, joydata.y_percentual);
            } else {
                ERROR_printf("✗ Erro na publicação MQTT: err1=%d, err2=%d\n", err1, err2);
            }
        } else {
            DEBUG_printf("Cliente MQTT desconectado. Dados não publicados.\n");
        }
    } else {
        DEBUG_printf("Nenhum dado disponível na fila MQTT do joystick\n");
    }
}

// Requisição de Assinatura - subscribe
void sub_request_cb(void *arg, err_t err) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    if (err != 0) {
        ERROR_printf("subscribe request failed %d", err);
        return;
    }
    state->subscribe_count++;
}

// Requisição para encerrar a assinatura
void unsub_request_cb(void *arg, err_t err) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    if (err != 0) {
        ERROR_printf("unsubscribe request failed %d", err);
        return;
    }
    state->subscribe_count--;
    if (state->subscribe_count < 0) state->subscribe_count = 0;

    // Stop if requested
    if (state->subscribe_count <= 0 && state->stop_client) {
        mqtt_disconnect(state->mqtt_client_inst);
    }
}

// Tópicos de assinatura
void sub_unsub_topics(MQTT_CLIENT_DATA_T* state, bool sub) {
    mqtt_request_cb_t cb = sub ? sub_request_cb : unsub_request_cb;
    
    // Tópicos de controle
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/led"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/print"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/ping"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/exit"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/control/buzzer"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/control/leds"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/control/alerts"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/control/matrix"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
}

// Dados de entrada MQTT
void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
#if MQTT_UNIQUE_TOPIC
    const char *basic_topic = state->topic + strlen(state->mqtt_client_info.client_id) + 1;
#else
    const char *basic_topic = state->topic;
#endif
    
    // Garantir que não exceda o buffer
    size_t copy_len = (len < sizeof(state->data) - 1) ? len : sizeof(state->data) - 1;
    strncpy(state->data, (const char *)data, copy_len);
    state->len = copy_len;
    state->data[copy_len] = '\0';

    DEBUG_printf("Topic: %s, Message: %s\n", state->topic, state->data);
    
    // Tópicos originais
    if (strcmp(basic_topic, "/led") == 0) {
        if (lwip_stricmp((const char *)state->data, "On") == 0 || strcmp((const char *)state->data, "1") == 0)
            control_led(state, true);
        else if (lwip_stricmp((const char *)state->data, "Off") == 0 || strcmp((const char *)state->data, "0") == 0)
            control_led(state, false);
    } 
    else if (strcmp(basic_topic, "/print") == 0) {
        INFO_printf("%.*s\n", (int)copy_len, data);
    } 
    else if (strcmp(basic_topic, "/ping") == 0) {
        char buf[11];
        snprintf(buf, sizeof(buf), "%u", to_ms_since_boot(get_absolute_time()) / 1000);
        mqtt_publish(state->mqtt_client_inst, full_topic(state, "/uptime"), buf, strlen(buf), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
    } 
    else if (strcmp(basic_topic, "/exit") == 0) {
        state->stop_client = true;
        sub_unsub_topics(state, false);
    }
    // NOVOS CONTROLES VIA MQTT
    else if (strcmp(basic_topic, "/control/buzzer") == 0) {
        if (lwip_stricmp((const char *)state->data, "enable") == 0 || 
            lwip_stricmp((const char *)state->data, "on") == 0 || 
            strcmp((const char *)state->data, "1") == 0) {
            set_buzzer_flag(true);
            INFO_printf("Buzzer habilitado via MQTT\n");
        }
        else if (lwip_stricmp((const char *)state->data, "disable") == 0 || 
                 lwip_stricmp((const char *)state->data, "off") == 0 || 
                 strcmp((const char *)state->data, "0") == 0) {
            set_buzzer_flag(false);
            INFO_printf("Buzzer desabilitado via MQTT\n");
        }
    }
    else if (strcmp(basic_topic, "/control/leds") == 0) {
        if (lwip_stricmp((const char *)state->data, "enable") == 0 || 
            lwip_stricmp((const char *)state->data, "on") == 0 || 
            strcmp((const char *)state->data, "1") == 0) {
            set_leds_flag(true);
            INFO_printf("LEDs habilitados via MQTT\n");
        }
        else if (lwip_stricmp((const char *)state->data, "disable") == 0 || 
                 lwip_stricmp((const char *)state->data, "off") == 0 || 
                 strcmp((const char *)state->data, "0") == 0) {
            set_leds_flag(false);
            INFO_printf("LEDs desabilitados via MQTT\n");
        }
    }
    else if (strcmp(basic_topic, "/control/alerts") == 0) {
        if (lwip_stricmp((const char *)state->data, "enable") == 0 || 
            lwip_stricmp((const char *)state->data, "on") == 0 || 
            strcmp((const char *)state->data, "1") == 0) {
            set_alerts_flag(true);
            INFO_printf("Sistema de alertas habilitado via MQTT\n");
        }
        else if (lwip_stricmp((const char *)state->data, "disable") == 0 || 
                 lwip_stricmp((const char *)state->data, "off") == 0 || 
                 strcmp((const char *)state->data, "0") == 0) {
            set_alerts_flag(false);
            INFO_printf("Sistema de alertas desabilitado via MQTT\n");
        }
    }else if (strcmp(basic_topic, "/control/matrix") == 0) {
        if (lwip_stricmp((const char *)state->data, "enable") == 0 || 
            lwip_stricmp((const char *)state->data, "on") == 0 || 
            strcmp((const char *)state->data, "1") == 0) {
            set_matrix_flag(true);
            INFO_printf("Matriz de LEDs habilitada via MQTT\n");
        }
        else if (lwip_stricmp((const char *)state->data, "disable") == 0 || 
                 lwip_stricmp((const char *)state->data, "off") == 0 || 
                 strcmp((const char *)state->data, "0") == 0) {
            set_matrix_flag(false);
            INFO_printf("Matriz de LEDs desabilitada via MQTT\n");
        }
    }
}

// Dados de entrada publicados
void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    strncpy(state->topic, topic, sizeof(state->topic) - 1);
    state->topic[sizeof(state->topic) - 1] = '\0';
}

// Conexão MQTT
void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    MQTT_CLIENT_DATA_T *state = (MQTT_CLIENT_DATA_T *)arg;

    if (status == MQTT_CONNECT_ACCEPTED) {
        INFO_printf("MQTT connection accepted\n");
        state->connect_done = true;

        sub_unsub_topics(state, true); // Assina tópicos

        if (state->mqtt_client_info.will_topic) {
            mqtt_publish(state->mqtt_client_inst,
                         state->mqtt_client_info.will_topic,
                         "1", 1, MQTT_WILL_QOS,
                         true, pub_request_cb, state);
        }

        INFO_printf("MQTT client conectado e pronto para uso\n");

    } else if (status == MQTT_CONNECT_DISCONNECTED) {
        INFO_printf("MQTT disconnected\n");
        state->connect_done = false;
    } else {
        ERROR_printf("MQTT connection error: status = %d\n", status);
        state->connect_done = false;
    }
}

// Inicializar o cliente MQTT
void start_client(MQTT_CLIENT_DATA_T *state) {
#if LWIP_ALTCP && LWIP_ALTCP_TLS
    const int port = MQTT_TLS_PORT;
    INFO_printf("Using TLS\n");
#else
    const int port = MQTT_PORT;
    INFO_printf("Warning: Not using TLS\n");
#endif

    state->mqtt_client_inst = mqtt_client_new();
    if (!state->mqtt_client_inst) {
        panic("MQTT client instance creation error");
    }
    INFO_printf("IP address of this device %s\n", ipaddr_ntoa(&(netif_list->ip_addr)));
    INFO_printf("Connecting to mqtt server at %s\n", ipaddr_ntoa(&state->mqtt_server_address));

    cyw43_arch_lwip_begin();
    if (mqtt_client_connect(state->mqtt_client_inst, &state->mqtt_server_address, port, mqtt_connection_cb, state, &state->mqtt_client_info) != ERR_OK) {
        panic("MQTT broker connection error");
    }
#if LWIP_ALTCP && LWIP_ALTCP_TLS
    // This is important for MBEDTLS_SSL_SERVER_NAME_INDICATION
    mbedtls_ssl_set_hostname(altcp_tls_context(state->mqtt_client_inst->conn), MQTT_SERVER);
#endif
    mqtt_set_inpub_callback(state->mqtt_client_inst, mqtt_incoming_publish_cb, mqtt_incoming_data_cb, state);
    cyw43_arch_lwip_end();
}

// Call back com o resultado do DNS
void dns_found(const char *hostname, const ip_addr_t *ipaddr, void *arg) {
    MQTT_CLIENT_DATA_T *state = (MQTT_CLIENT_DATA_T*)arg;
    if (ipaddr) {
        state->mqtt_server_address = *ipaddr;
        start_client(state);
    } else {
        panic("dns request failed");
    }
}

void reconnect_mqtt(MQTT_CLIENT_DATA_T *state) {
    INFO_printf("Attempting MQTT reconnection...\n");

    // Limpar estado anterior
    state->connect_done = false;
    
    // Resolve novamente o IP do servidor MQTT
    cyw43_arch_lwip_begin();
    int err = dns_gethostbyname(MQTT_SERVER, &state->mqtt_server_address, dns_found, state);
    cyw43_arch_lwip_end();

    if (err == ERR_OK) {
        start_client(state); // reconecta imediatamente
    } else if (err != ERR_INPROGRESS) {
        ERROR_printf("DNS request failed during reconnection\n");
    }
}
