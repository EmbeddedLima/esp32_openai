/*
*	Teste chat GPT com ESP32
*/

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "OpenAI.h"

static const char *TAG = "app_main";
extern void wifi_init_sta();

const char *inputText = "Em qual horario podemos almocar?";

#define OPEN_AI_PATTERN_CHR_NUM (3)

#define OPEN_AI_BUF_SIZE 				(1024)
#define OPEN_AI_RD_BUF_SIZE 		(OPEN_AI_BUF_SIZE)

#define OPEN_AI_UART_NUM 				UART_NUM_1
#define OPEN_AI_UART_TX_NUM 		GPIO_NUM_17
#define OPEN_AI_UART_RX_NUM 		GPIO_NUM_18
#define OPEN_AI_UART_BUF_SIZE 	(16384)

static QueueHandle_t gprs_uart_queue;

void chat_response(const char *inputText)
{
    // Cria uma instância da estrutura OpenAI_t
    OpenAI_t *openai = OpenAICreate(CONFIG_OPENAI_KEY);

    // Cria uma instância da estrutura OpenAI_ChatCompletion_t
    OpenAI_ChatCompletion_t *chatCompletion = openai->chatCreate(openai);

    // Configura o modelo a ser usado
    chatCompletion->setModel(chatCompletion, "gpt-3.5-turbo");

    // Define o número máximo de tokens na resposta
    chatCompletion->setMaxTokens(chatCompletion, 500);

    // Configura a temperatura para controlar a aleatoriedade das respostas
    chatCompletion->setTemperature(chatCompletion, 0.2);

    // Configura o caractere de parada da resposta (neste caso, uma quebra de linha)
    chatCompletion->setStop(chatCompletion, "\r");

    // Define a penalidade de presença (presence penalty) para 0
    chatCompletion->setPresencePenalty(chatCompletion, 0);

    // Define a penalidade de frequência (frequency penalty) para 0
    chatCompletion->setFrequencyPenalty(chatCompletion, 0);

    // Define o usuário que está fazendo a pergunta
    chatCompletion->setUser(chatCompletion, "OpenAI-ESP32");

    // Cria uma instância da estrutura OpenAI_StringResponse_t para armazenar a resposta
    OpenAI_StringResponse_t *result = chatCompletion->message(chatCompletion, inputText, false);

    // Obtém a resposta em forma de texto
    char *response = result->getData(result, 0);

    // Registra a pergunta no log
    ESP_LOGI(TAG, "----------------------------------------\n");
    ESP_LOGI(TAG, "Pergunta: %s", inputText);

    // Registra a resposta no log
    ESP_LOGW(TAG, "Resposta: %s\n", response);
    ESP_LOGI(TAG, "----------------------------------------\n");
}

static void uart_event_task(void *pvParameters)
{
    uart_event_t event = {};
    uint8_t *dtmp = (uint8_t *)malloc(OPEN_AI_UART_BUF_SIZE);

    while (1)
    {
        // Esperando por evento UART.
        if (xQueueReceive(gprs_uart_queue, (void *)&event, (TickType_t)portMAX_DELAY))
        {
            bzero(dtmp, OPEN_AI_UART_BUF_SIZE);
            ESP_LOGI(TAG, "uart[%d] event:", OPEN_AI_UART_NUM);

            switch (event.type)
            {
            case UART_DATA:
            {
                if (event.size > 0 && event.size < 3000) // 3 KB
                {
                    char *buff_in = (char *)malloc(event.size);
                    bzero(buff_in, event.size);

                    int char_position = 0;
                    int aux = 0;

                    // Evento de recebimento de dados UART
                    uart_read_bytes(OPEN_AI_UART_NUM, dtmp, event.size, portMAX_DELAY);
                    ESP_LOGI(TAG, "Tamanho do evento: %d", event.size);

                    for (char_position = 0; char_position < event.size; char_position++)
                    {
                        // 0x7F = ASCII space
                        // 0x1A <= Valores ASCII C^
                        if (dtmp[char_position] != 0x7F && dtmp[char_position] > 0x1A)
                        {
                            buff_in[aux] = dtmp[char_position];
                            aux++;
                        }
                    }

                    chat_response((char *)dtmp);
                    free(buff_in);
                }
                break;
            }
            case UART_FIFO_OVF:
            {
                // Evento de overflow de FIFO de hardware detectado
                ESP_LOGW(TAG, "Overflow de FIFO de hardware");
                uart_flush_input(OPEN_AI_UART_NUM);
                xQueueReset(gprs_uart_queue);

                break;
            }
            case UART_BUFFER_FULL:
            {
                // Evento de buffer de anel UART cheio
                ESP_LOGW(TAG, "Buffer de anel cheio");
                uart_flush_input(OPEN_AI_UART_NUM);
                xQueueReset(gprs_uart_queue);

                break;
            }
            case UART_BREAK:
            {
                // Evento de detecção de quebra de RX UART
                ESP_LOGI(TAG, "Quebra de RX UART");

                break;
            }
            case UART_PARITY_ERR:
            {
                // Evento de erro de verificação de paridade UART
                ESP_LOGE(TAG, "%s, Erro de verificação de paridade UART", __func__);

                break;
            }
            case UART_FRAME_ERR:
            {
                // Evento de erro de quadro UART
                ESP_LOGE(TAG, "%s, Erro de quadro UART", __func__);

                break;
            }
            case UART_PATTERN_DET:
            {
                // Evento de detecção de padrão UART
                ESP_LOGW(TAG, "%s, Padrão UART", __func__);

                break;
            }
            default:
            {
                // Outros eventos
                ESP_LOGW(TAG, "Tipo de evento UART: %d", event.type);

                break;
            }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void app_main(void)
{
    // Inicializa o NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();

    /* Configure os parâmetros de um driver UART,
     * pinos de comunicação e instala o driver */
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // Instala o driver UART e obtém a fila.
    uart_driver_install(OPEN_AI_UART_NUM, OPEN_AI_UART_BUF_SIZE * 6, OPEN_AI_UART_BUF_SIZE * 2, 20, &gprs_uart_queue, 0);
    uart_param_config(OPEN_AI_UART_NUM, &uart_config);

    // Define o nível de log UART
    esp_log_level_set(TAG, ESP_LOG_INFO);
    // Define os pinos UART (usando os pinos padrão da UART0)
    uart_set_pin(OPEN_AI_UART_NUM, OPEN_AI_UART_TX_NUM, OPEN_AI_UART_RX_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // Configura a função de detecção de padrão UART.
    uart_enable_pattern_det_baud_intr(OPEN_AI_UART_NUM, '+', OPEN_AI_PATTERN_CHR_NUM, 9, 0, 0);
    // Redefine o comprimento da fila de padrões para registrar no máximo 20 posições de padrões.
    uart_pattern_queue_reset(OPEN_AI_UART_NUM, 20);

    // Cria uma tarefa para manipular eventos UART a partir da ISR
    xTaskCreate(uart_event_task, "uart_event_task", configMINIMAL_STACK_SIZE * 4, NULL, 12, NULL);

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
