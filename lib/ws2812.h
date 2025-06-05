#pragma once

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "ws2812.pio.h"             // Programa para controle de LEDs WS2812B

#define LED_COUNT 25
#define LED_PIN 7


struct pixel_t { 
    uint32_t G, R, B;                // Componentes de cor: Verde, Vermelho e Azul
};


typedef struct pixel_t pixel_t;     // Alias para a estrutura pixel_t
typedef pixel_t npLED_t;            // Alias para facilitar o uso no contexto de LEDs

npLED_t leds[LED_COUNT];            // Array para armazenar o estado de cada LED
PIO np_pio;                         // Variável para referenciar a instância PIO usada
uint sm;                            // Variável para armazenar o número do state machine usado


int getIndex(int x, int y) {
    // Se a linha for par (0, 2, 4), percorremos da esquerda para a direita.
    // Se a linha for ímpar (1, 3), percorremos da direita para a esquerda.
    if (y % 2 == 0) {
        return 24-(y * 5 + x); // Linha par (esquerda para direita).
    } else {
        return 24-(y * 5 + (4 - x)); // Linha ímpar (direita para esquerda).
    }
}

// Função para inicializar o PIO para controle dos LEDs
void npInit(uint pin) 
{
    uint offset = pio_add_program(pio0, &ws2812_program); // Carregar o programa PIO
    np_pio = pio0;                                         // Usar o primeiro bloco PIO

    sm = pio_claim_unused_sm(np_pio, false);              // Tentar usar uma state machine do pio0
    if (sm < 0)                                           // Se não houver disponível no pio0
    {
        np_pio = pio0;                                    // Mudar para o pio0
        sm = pio_claim_unused_sm(np_pio, true);           // Usar uma state machine do pio0
    }

    ws2812_program_init(np_pio, sm, offset, pin, 800000.f); // Inicializar state machine para LEDs

    for (uint i = 0; i < LED_COUNT; ++i)                  // Inicializar todos os LEDs como apagados
    {
        leds[i].R = 0;
        leds[i].G = 0;
        leds[i].B = 0;
    }
}

// Função para definir a cor de um LED específico
void npSetLED(const uint index, const uint8_t r, const uint8_t g, const uint8_t b) 
{
    leds[index].R = r;                                    // Definir componente vermelho
    leds[index].G = g;                                    // Definir componente verde
    leds[index].B = b;                                    // Definir componente azul
}

// Função para atualizar os LEDs no hardware
void npWrite() 
{
    for (uint i = 0; i < LED_COUNT; ++i)                  // Iterar sobre todos os LEDs
    {
        pio_sm_put_blocking(np_pio, sm, leds[i].G<<24);       // Enviar componente verde
        pio_sm_put_blocking(np_pio, sm, leds[i].R<<24);       // Enviar componente vermelho
        pio_sm_put_blocking(np_pio, sm, leds[i].B<<24);       // Enviar componente azul
    }
}

// Função para limpar (apagar) todos os LEDs
void npClear() 
{
    for (uint i = 0; i < LED_COUNT; ++i)                  // Iterar sobre todos os LEDs
      {  npSetLED(i, 0, 0, 0);    }                        // Definir cor como preta (apagado)

    npWrite();
}

void print_frame(int frame[5][5], float red_brightness, float green_brightness, float blue_brightness)
{
    for(int linha = 0; linha < 5; linha++){
        for(int coluna = 0; coluna < 5; coluna++){
            int posicao = getIndex(linha, coluna);
            npSetLED(posicao, frame[coluna][linha]*red_brightness, frame[coluna][linha]*green_brightness, frame[coluna][linha]*blue_brightness);
        }
    }
    npWrite();
    //sleep_ms(sleep_time);
   // npClear(); 
}


int frame0[5][5] = {
                {0, 1, 1, 1, 0},
                {0, 1, 0, 1, 0},
                {0, 1, 0, 1, 0},
                {0, 1, 0, 1, 0},
                {0, 1, 1, 1, 0}
                };

int frame1[5][5] = {
                {0, 0, 1, 0, 0},
                {0, 1, 1, 0, 0},
                {0, 0, 1, 0, 0},
                {0, 0, 1, 0, 0},
                {0, 1, 1, 1, 0}
                };

int frame2[5][5] = {
                {0, 1, 1, 0, 0},
                {1, 0, 0, 1, 0},
                {0, 0, 1, 0, 0},
                {0, 1, 0, 0, 0},
                {1, 1, 1, 1, 0}
                };

int frame3[5][5] = {
                {0, 1, 1, 1, 1},
                {0, 0, 0, 0, 1},
                {0, 0, 1, 1, 1},
                {0, 0, 0, 0, 1},
                {0, 1, 1, 1, 1}
                };

int frame4[5][5] = {
                {0, 0, 0, 1, 1},
                {0, 0, 1, 0, 1},
                {0, 1, 1, 1, 1},
                {0, 0, 0, 0, 1},
                {0, 0, 0, 0, 1}
                };

int frame5[5][5] = {
                {0, 1, 1, 1, 1},
                {0, 1, 0, 0, 0},
                {0, 1, 1, 1, 1},
                {0, 0, 0, 0, 1},
                {0, 1, 1, 1, 1}
                };

int frame6[5][5] = {
                {0, 1, 1, 1, 1},
                {0, 1, 0, 0, 0},
                {0, 1, 1, 1, 1},
                {0, 1, 0, 0, 1},
                {0, 1, 1, 1, 1}
                };

int frame7[5][5] = {
                {0, 1, 1, 1, 1},
                {0, 0, 0, 0, 1},
                {0, 0, 0, 1, 0},
                {0, 0, 1, 0, 0},
                {0, 1, 0, 0, 0}
                };

int frame8[5][5] = {
                {0, 1, 1, 1, 0},
                {0, 1, 0, 1, 0},
                {0, 1, 1, 1, 0},
                {0, 1, 0, 1, 0},
                {0, 1, 1, 1, 0}
                };

int frame9[5][5] = {
                {0, 1, 1, 1, 1},
                {0, 1, 0, 0, 1},
                {0, 1, 1, 1, 1},
                {0, 0, 0, 0, 1},
                {0, 1, 1, 1, 1}
                };

