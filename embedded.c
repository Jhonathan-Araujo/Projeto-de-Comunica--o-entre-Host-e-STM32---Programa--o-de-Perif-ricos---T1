// Trabalho 1 da disciplina de programação de periféricos
// Desenvolvido por Jhonathan Araújo Mesquita de Freitas e Tomas Haddad Caldas
// Prof. Dr. Sérgio Johann Filho
// PUCRS - Pontifícia Universidade Católica do Rio Grande do Sul
// Código do dispositivo embarcado STM32

#include <stm32f4xx.h>
#include <stm32f4xx_conf.h>
#include <hal.h>
#include <usart.h>
#include <libc.h>
#include "usbd_cdc_vcp.h"

// Definições de pinos
#define INPUT_PINS (GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6)
#define OUTPUT_PINS (GPIO_Pin_7 | GPIO_Pin_6 | GPIO_Pin_5 | GPIO_Pin_4)

#define LED1_PIN GPIO_Pin_7
#define LED2_PIN GPIO_Pin_6
#define LED3_PIN GPIO_Pin_5
#define LED4_PIN GPIO_Pin_4

#define LED_PORT GPIOA

#define SW1_PIN GPIO_Pin_3
#define SW2_PIN GPIO_Pin_4
#define SW3_PIN GPIO_Pin_5
#define SW4_PIN GPIO_Pin_6

#define SW_PORT GPIOB

// Endereços dos recursos
#define ADDR_LED1 0x0001
#define ADDR_LED2 0x0002
#define ADDR_LED3 0x0003
#define ADDR_LED4 0x0004

#define ADDR_SW1 0x0101
#define ADDR_SW2 0x0102
#define ADDR_SW3 0x0103
#define ADDR_SW4 0x0104

#define ADDR_SW1_TRIGGER 0x0201
#define ADDR_SW2_TRIGGER 0x0202
#define ADDR_SW3_TRIGGER 0x0203
#define ADDR_SW4_TRIGGER 0x0204

// Tipos de gatilho
#define TRIGGER_NONE     0x0000
#define TRIGGER_RISING   0x0001
#define TRIGGER_FALLING  0x0002
#define TRIGGER_BOTH     0x0003

// Variáveis globais
uint16_t sw_trigger_config[4] = {TRIGGER_NONE, TRIGGER_NONE, TRIGGER_NONE, TRIGGER_NONE}; // Estados de gatilho possíveis

// Declaração da função send_response
void send_response(uint16_t TID, uint8_t OPER, uint16_t ADDR, uint16_t DATA);

// Funções auxiliares para manipulação de memória
void my_memcpy(uint8_t *dest, const uint8_t *src, uint16_t len) // Função adicionada por conta de problemas com a memcpy nativa
{
    while (len--)
    {
        *dest++ = *src++;
    }
}

void my_memset(uint8_t *dest, uint8_t val, uint16_t len)
{
    while (len--)
    {
        *dest++ = val;
    }
}

// Função para configurar GPIO
void setupGPIO(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // Habilitar o clock para GPIOA (saídas) e GPIOB (entradas)
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    // Configurar os pinos de saída (LEDs)
    GPIO_InitStructure.GPIO_Pin = OUTPUT_PINS;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(LED_PORT, &GPIO_InitStructure);

    // Configurar os pinos de entrada (chaves) com pull-up interno
    GPIO_InitStructure.GPIO_Pin = INPUT_PINS;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; // Ativar resistores de pull-up internos
    GPIO_Init(SW_PORT, &GPIO_InitStructure);
}

// Função para configurar interrupções EXTI
void setupEXTI(void)
{
    EXTI_InitTypeDef   EXTI_InitStructure;
    NVIC_InitTypeDef   NVIC_InitStructure;

    // Habilitar o clock para SYSCFG
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    // Configurar EXTI para SW1 (PB3)
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource3);
    EXTI_InitStructure.EXTI_Line = EXTI_Line3;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_LineCmd = DISABLE; // Desabilitado inicialmente
    EXTI_Init(&EXTI_InitStructure);

    // Configurar EXTI para SW2 (PB4)
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource4);
    EXTI_InitStructure.EXTI_Line = EXTI_Line4;
    EXTI_InitStructure.EXTI_LineCmd = DISABLE; // Desabilitado inicialmente
    EXTI_Init(&EXTI_InitStructure);

    // Configurar EXTI para SW3 (PB5)
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource5);
    EXTI_InitStructure.EXTI_Line = EXTI_Line5;
    EXTI_InitStructure.EXTI_LineCmd = DISABLE; // Desabilitado inicialmente
    EXTI_Init(&EXTI_InitStructure);

    // Configurar EXTI para SW4 (PB6)
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource6);
    EXTI_InitStructure.EXTI_Line = EXTI_Line6;
    EXTI_InitStructure.EXTI_LineCmd = DISABLE; // Desabilitado inicialmente
    EXTI_Init(&EXTI_InitStructure);

    // Configurar prioridades e habilitar interrupções EXTI
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_Init(&NVIC_InitStructure);
}

// Função para atualizar a configuração de gatilho EXTI
void update_EXTI_trigger(uint8_t sw_index)
{
    EXTI_InitTypeDef EXTI_InitStructure;

    uint32_t EXTI_Line;
    switch(sw_index)
    {
        case 0: EXTI_Line = EXTI_Line3; break;
        case 1: EXTI_Line = EXTI_Line4; break;
        case 2: EXTI_Line = EXTI_Line5; break;
        case 3: EXTI_Line = EXTI_Line6; break;
        default: return; // Índice inválido
    }

    // Configurar EXTI
    EXTI_InitStructure.EXTI_Line = EXTI_Line;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;

    if (sw_trigger_config[sw_index] == TRIGGER_NONE)
    {
        EXTI_InitStructure.EXTI_LineCmd = DISABLE;
    }
    else
    {
        EXTI_InitStructure.EXTI_LineCmd = ENABLE;

        switch(sw_trigger_config[sw_index])
        {
            case TRIGGER_RISING:
                EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; // Borda de subida
                break;
            case TRIGGER_FALLING:
                EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; // Borda de descica
                break;
            case TRIGGER_BOTH:
                EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; // Borda de subida e de descida (ambas)
                break;
            default:
                EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; // Padrão: Borda de subida e de descida (ambas)
                break;
        }
    }

    EXTI_Init(&EXTI_InitStructure);
}

// Funções de manipulação de eventos de entrada
void handle_input_event(uint8_t sw_index)
{
    // Verificar se o gatilho está configurado
    if (sw_trigger_config[sw_index] != TRIGGER_NONE)
    {
        uint16_t pin;
        switch (sw_index)
        {
            case 0: pin = SW1_PIN; break;
            case 1: pin = SW2_PIN; break;
            case 2: pin = SW3_PIN; break;
            case 3: pin = SW4_PIN; break;
            default: return; // Índice inválido
        }

        // Ler o estado atual do pino
        uint8_t pin_state = GPIO_ReadInputDataBit(SW_PORT, pin);

        // Enviar mensagem assíncrona
        uint16_t TID = 0x0000;
        uint8_t OPER = 0x00; // Operação de leitura
        uint16_t ADDR = ADDR_SW1 + sw_index;
        uint16_t DATA = pin_state;

        send_response(TID, OPER, ADDR, DATA);
    }
}

// Manipuladores de interrupção EXTI
void EXTI3_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line3) != RESET)
    {
        handle_input_event(0); // SW1
        EXTI_ClearITPendingBit(EXTI_Line3);
    }
}

void EXTI4_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line4) != RESET)
    {
        handle_input_event(1); // SW2
        EXTI_ClearITPendingBit(EXTI_Line4);
    }
}

void EXTI9_5_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line5) != RESET)
    {
        handle_input_event(2); // SW3
        EXTI_ClearITPendingBit(EXTI_Line5);
    }
    if (EXTI_GetITStatus(EXTI_Line6) != RESET)
    {
        handle_input_event(3); // SW4
        EXTI_ClearITPendingBit(EXTI_Line6);
    }
}

// Funções de byte stuffing e unstuffing
void byte_stuffing(uint8_t *input, uint16_t input_len, uint8_t *output, uint16_t *output_len)
{
    uint16_t i, j = 0;
    for (i = 0; i < input_len; i++)
    {
        if (input[i] == 0x7e || input[i] == 0x7d)
        {
            output[j++] = 0x7d;
            output[j++] = input[i] ^ 0x20; // XOR
        }
        else
        {
            output[j++] = input[i];
        }
    }
    *output_len = j;
}

int byte_unstuffing(uint8_t *input, uint16_t input_len, uint8_t *output, uint16_t *output_len)
{
    uint16_t i = 0, j = 0;
    while (i < input_len)
    {
        if (input[i] == 0x7d)
        {
            i++;
            if (i >= input_len)
                return -1; // Erro
            output[j++] = input[i++] ^ 0x20;
        }
        else
        {
            output[j++] = input[i++];
        }
    }
    *output_len = j;
    return 0;
}

// Função para enviar resposta
void send_response(uint16_t TID, uint8_t OPER, uint16_t ADDR, uint16_t DATA)
{
    uint8_t msg[7];
    uint8_t stuffed_msg[14]; // Tamanho máximo depois do  stuffing
    uint16_t stuffed_len;
    uint8_t frame[16]; // Quadro com delimitadores
    uint16_t frame_len = 0;

    // Preparo do conteúdo do quadro
    msg[0] = (TID >> 8) & 0xFF;
    msg[1] = TID & 0xFF;
    msg[2] = OPER;
    msg[3] = (ADDR >> 8) & 0xFF;
    msg[4] = ADDR & 0xFF;
    msg[5] = (DATA >> 8) & 0xFF;
    msg[6] = DATA & 0xFF;

    // Aplicar byte stuffing
    byte_stuffing(msg, 7, stuffed_msg, &stuffed_len);

    // Delimitador incial
    frame[frame_len++] = 0x7E;

    // Mensagem "estufada"
    my_memcpy(&frame[frame_len], stuffed_msg, stuffed_len);
    frame_len += stuffed_len;

    // Delimitador final
    frame[frame_len++] = 0x7E;

    // Enviar o quadro via USB CDC
    for (uint16_t i = 0; i < frame_len; i++)
    {
        VCP_putchar(frame[i]);
    }
}

// Função para receber mensagens
int receive_message(uint8_t *buf, uint16_t *len)
{
    static uint8_t rx_buf[256];
    static uint16_t rx_len = 0;
    uint8_t byte;
    uint8_t unstuffed_buf[256];
    uint16_t unstuffed_len;
    static uint8_t in_frame = 0;

    // Ler dado do USB CDC
    while (VCP_getchar(&byte))
    {
        //GPIO_SetBits(GPIOA, GPIO_Pin_7); // Ligar o LED A7 para depurar se algo foi recebido
        if (byte == 0x7E)
        {
            //GPIO_SetBits(GPIOA, GPIO_Pin_6); // Ligar o LED A6 para depurar se um delimitador foi recebido
            if (in_frame)
            {
                // Fim do quadro
                if (byte_unstuffing(rx_buf, rx_len, unstuffed_buf, &unstuffed_len) != 0)
                {
                    // Erro ao "desestufar"
                    rx_len = 0;
                    in_frame = 0;
                    return -1;
                }
                my_memcpy(buf, unstuffed_buf, unstuffed_len);
                *len = unstuffed_len;
                rx_len = 0;
                in_frame = 0;
                //GPIO_SetBits(GPIOA, GPIO_Pin_4); // Ligar o LED A4 para depurar se o frame foi inteiramente recebido
                return 0; // Mensagem recebida com sucesso
            }
            else
            {
                //GPIO_SetBits(GPIOA, GPIO_Pin_5); // Ligar o LED A5 para depurar se um frame começou a ser recebido
                // Começo do quadro
                in_frame = 1;
                rx_len = 0;
            }
        }
        else if (in_frame)
        {
            if (rx_len < sizeof(rx_buf))
            {
                rx_buf[rx_len++] = byte;
            }
            else
            {
                // Overflow do buffer
                rx_len = 0;
                in_frame = 0;
                return -1;
            }
        }
    }
    return -1; // Mensagem ainda incompleta
}
// Função para processar comandos
void process_command(uint16_t TID, uint8_t OPER, uint16_t ADDR, uint16_t DATA)
{
    uint16_t response_DATA = 0;
    uint8_t response_OPER = OPER;
    //GPIO_SetBits(GPIOA, GPIO_Pin_7); // Ligar o LED A7 (depuração)
    if (OPER == 0) // Operação de leitura
    {
        //GPIO_SetBits(GPIOA, GPIO_Pin_7); // Ligar o LED A7 (depuração)
        switch (ADDR)
        {
            case ADDR_LED1:
            case ADDR_LED2:
            case ADDR_LED3:
            case ADDR_LED4:
            {
                uint16_t pin = 0;
                switch (ADDR)
                {
                    case ADDR_LED1: pin = LED1_PIN; break;
                    case ADDR_LED2: pin = LED2_PIN; break;
                    case ADDR_LED3: pin = LED3_PIN; break;
                    case ADDR_LED4: pin = LED4_PIN; break;
                }
                uint8_t state = GPIO_ReadOutputDataBit(LED_PORT, pin);
                response_DATA = state;
                break;
            }
            case ADDR_SW1:
            case ADDR_SW2:
            case ADDR_SW3:
            case ADDR_SW4:
            {
                uint16_t pin = 0;
                switch (ADDR)
                {
                    case ADDR_SW1: pin = SW1_PIN; break;
                    case ADDR_SW2: pin = SW2_PIN; break;
                    case ADDR_SW3: pin = SW3_PIN; break;
                    case ADDR_SW4: pin = SW4_PIN; break;
                }
                uint8_t state = GPIO_ReadInputDataBit(SW_PORT, pin);
                response_DATA = state;
                break;
            }
            case ADDR_SW1_TRIGGER:
            case ADDR_SW2_TRIGGER:
            case ADDR_SW3_TRIGGER:
            case ADDR_SW4_TRIGGER:
            {
                uint8_t sw_index = ADDR - ADDR_SW1_TRIGGER;
                response_DATA = sw_trigger_config[sw_index];
                break;
            }
            default:
                response_DATA = 0xFFFF; // Endereço inválido
                break;
        }
    }
    else if (OPER == 1) // Operação de escrita
    {
        //GPIO_SetBits(GPIOA, GPIO_Pin_6); // Ligar o LED A6 (depuração)
        switch (ADDR)
        {
            case ADDR_LED1:
            case ADDR_LED2:
            case ADDR_LED3:
            case ADDR_LED4:
            {
                uint16_t pin = 0;
                switch (ADDR)
                {
                    case ADDR_LED1: pin = LED1_PIN; break;
                    case ADDR_LED2: pin = LED2_PIN; break;
                    case ADDR_LED3: pin = LED3_PIN; break;
                    case ADDR_LED4: pin = LED4_PIN; break;
                }
                if (DATA == 0)
                    GPIO_ResetBits(LED_PORT, pin);
                else
                    GPIO_SetBits(LED_PORT, pin);
                response_DATA = DATA;
                break;
            }
            case ADDR_SW1_TRIGGER:
            case ADDR_SW2_TRIGGER:
            case ADDR_SW3_TRIGGER:
            case ADDR_SW4_TRIGGER:
            {
                uint8_t sw_index = ADDR - ADDR_SW1_TRIGGER;
                if (DATA >= 0 && DATA <= 3)
                {
                    sw_trigger_config[sw_index] = DATA;
                    update_EXTI_trigger(sw_index);
                    response_DATA = DATA;
                }
                else
                {
                    response_DATA = 0xFFFF; // Valor inválido
                }
                break;
            }
            default:
                response_DATA = 0xFFFF; // Endereço inválido
                break;
        }
    }
    else
    {
        response_OPER = 0xFF; // Operação inválida
    }

    // Enviar resposta
    send_response(TID, response_OPER, ADDR, response_DATA);
}

// Função principal
int main(void)
{
    uint8_t buf[256];
    uint16_t len;

    SystemInit();
    setupGPIO();
    setupEXTI();
    uart_init(0, 115200, 0);

    // GPIO_SetBits(GPIOA, GPIO_Pin_7); // Ligar o LED A7 (depuração)
    

    while (1)
    {
        if (receive_message(buf, &len) == 0)
        {
            
            if (len == 7)
            {
                // GPIO_SetBits(GPIOA, GPIO_Pin_6); // Ligar o LED A6 (depuração)
                uint16_t TID = (buf[0] << 8) | buf[1];
                uint8_t OPER = buf[2];
                uint16_t ADDR = (buf[3] << 8) | buf[4];
                uint16_t DATA = (buf[5] << 8) | buf[6];
                // GPIO_SetBits(GPIOA, GPIO_Pin_5); // Ligar o LED A5 (depuração)
                process_command(TID, OPER, ADDR, DATA);
                // GPIO_SetBits(GPIOA, GPIO_Pin_4); // Ligar o LED A4 (depuração)
            }
            else
            {
                // Mensagem com tamanho inválido
            }
        }
    }

    return 0;
}