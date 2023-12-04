# Projeto de Sistemas em Tempo Real

### Professor: Felipe Viel
### Acadêmicos: Bruno Jaciel de Mello e Sérgio Venturi

## Descrição do Projeto

Este projeto visa desenvolver um sistema de monitoramento e controle de subsistemas de um veículo utilizando a placa de desenvolvimento ESP32, atendendo a requisitos temporais específicos. O software foi desenvolvido em C e utiliza multithreading para melhorar o desempenho e garantir a qualidade do monitoramento e controle dos subsistemas.

## Requisitos

- C
- Bibliotecas: <stdio.h>
              "freertos/FreeRTOS.h"
              "freertos/task.h"
              "driver/touch_pad.h"
              "freertos/queue.h"
              "soc/rtc_periph.h"
              "soc/sens_periph.h"
              "esp_log.h"
              "freertos/semphr.h"
              "esp_timer.h"
- ESP32
