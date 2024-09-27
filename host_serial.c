// Trabalho 1 da disciplina de programação de periféricos
// Desenvolvido por Jhonathan Araújo Mesquita de Freitas e Tomas Haddad Caldas
// Prof. Dr. Sérgio Johann Filho
// PUCRS - Pontifícia Universidade Católica do Rio Grande do Sul
// Código da máquina host

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdint.h>
#include <time.h>
#include <string.h>
#include <errno.h>
#include <sys/select.h>
#include <pthread.h>

// Definições de constantes
#define FRAME_DELIMITER 0x7E
#define ESCAPE_CHAR 0x7D
#define XOR_VALUE 0x20

#define OP_READ 0x00
#define OP_WRITE 0x01

#define MAX_BUFFER_SIZE 256

// Endereços correspondentes aos definidos no STM32
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
#define TRIGGER_NONE 0x0000
#define TRIGGER_RISING 0x0001
#define TRIGGER_FALLING 0x0002
#define TRIGGER_BOTH 0x0003

// Estrutura de comando
typedef struct
{
    uint16_t tid;
    uint8_t oper;
    uint16_t addr;
    uint16_t data;
} Command;

// Variáveis globais
int fd;
volatile int running = 1;
pthread_t reader_thread;
pthread_mutex_t print_mutex = PTHREAD_MUTEX_INITIALIZER;

#define READ_BUF_SIZE 128
#define WRITE_BUF_SIZE 128

// Protótipos de funções
int configure_serial_port(int fd);
void process_received_data(uint8_t *data, uint16_t length);
void send_command(int fd, Command *cmd);
void byte_stuffing(uint8_t *input, uint16_t length, uint8_t *output, uint16_t *out_length);
int byte_unstuffing(uint8_t *input, uint16_t length, uint8_t *output, uint16_t *out_length);
uint16_t generate_tid();
void user_interface();
void cleanup();
void *asynchronous_reader(void *arg);

// Função principal
int main(int argc, char **argv)
{
    const char *device = "/dev/cu.usbmodem00000000050C1"; // Dispositivo serial (macOS)
    fd = open(device, O_RDWR | O_NOCTTY | O_SYNC);

    if (fd < 0)
    {
        perror("Erro ao abrir a porta serial");
        return 1;
    }

    if (configure_serial_port(fd) < 0)
    {
        close(fd);
        return 1;
    }

    printf("Aplicação de Controle - Host\n");

    // Semente para o gerador de números aleatórios
    srand((unsigned int)time(NULL));

    // Criar a thread para ler mensagens assíncronas
    if (pthread_create(&reader_thread, NULL, asynchronous_reader, NULL) != 0)
    {
        perror("Erro ao criar a thread de leitura assíncrona");
        close(fd);
        return 1;
    }

    // Loop principal
    while (running)
    {
        // Mostrar a interface do usuário
        user_interface();
    }

    // Esperar a thread de leitura terminar
    pthread_cancel(reader_thread);
    pthread_join(reader_thread, NULL);

    cleanup();
    return EXIT_SUCCESS;
}

// Função para configurar a porta serial
int configure_serial_port(int fd)
{
    struct termios tty;

    // Obter a configuração atual da interface serial
    if (tcgetattr(fd, &tty) != 0)
    {
        perror("Erro em tcgetattr");
        return -1;
    }

    // Configurar taxa de baud
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    // Modo raw
    cfmakeraw(&tty);

    // Sem controle de fluxo de hardware
    tty.c_cflag &= ~CRTSCTS;

    // Habilitar receptor e definir modo local
    tty.c_cflag |= (CLOCAL | CREAD);

    // Limpar o buffer de entrada
    tcflush(fd, TCIFLUSH);

    // Definir os novos atributos
    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        perror("Erro em tcsetattr");
        return -1;
    }

    return 0;
}

// Função para processar dados recebidos
void process_received_data(uint8_t *data, uint16_t length)
{
    pthread_mutex_lock(&print_mutex); // Aciona bloqueio do mutex

    if (length < 7)
    {
        printf("Quadro inválido recebido.\n");
        pthread_mutex_unlock(&print_mutex); // Aciona desbloqueio do mutex
        return;
    }

    // Extrair campos
    Command cmd;
    uint16_t index = 0;

    cmd.tid = (data[index++] << 8);
    cmd.tid |= data[index++];

    cmd.oper = data[index++];

    cmd.addr = (data[index++] << 8);
    cmd.addr |= data[index++];

    cmd.data = (data[index++] << 8);
    cmd.data |= data[index++];

    if (cmd.tid == 0x0000)
    {
        // Mensagem assíncrona
        printf("\n[Notificação] Evento no endereço 0x%04X: valor = %d\n", cmd.addr, cmd.data);
    }
    else
    {
        // Resposta a uma solicitação
        printf("\n[Resposta] TID: %d, Operação: %s, Endereço: 0x%04X, Valor: %d\n",
               cmd.tid,
               cmd.oper == OP_READ ? "Leitura" : "Escrita",
               cmd.addr,
               cmd.data);
    }

    pthread_mutex_unlock(&print_mutex);  // Aciona desbloqueio do mutex
}

// Função para enviar um comando e receber a resposta
void send_command(int fd, Command *cmd)
{
    uint8_t buffer[WRITE_BUF_SIZE];
    uint8_t stuffed_buffer[WRITE_BUF_SIZE * 2];  // Byte-stuffing pode aumentar o tamanho
    uint16_t index = 0;
    int bytes_written = 0;
    uint16_t stuffed_length = 0;

    // Construir o comando no buffer
    buffer[index++] = (cmd->tid >> 8) & 0xFF;
    buffer[index++] = cmd->tid & 0xFF;
    buffer[index++] = cmd->oper;
    buffer[index++] = (cmd->addr >> 8) & 0xFF;
    buffer[index++] = cmd->addr & 0xFF;
    buffer[index++] = (cmd->data >> 8) & 0xFF;
    buffer[index++] = cmd->data & 0xFF;

    // Aplicar byte stuffing
    byte_stuffing(buffer, index, stuffed_buffer, &stuffed_length);

    // Calcular o comprimento total do quadro final (delimitador inicial + dados com stuffing + delimitador final)
    uint16_t frame_length = 1 + stuffed_length + 1;  // Delimitador inicial, dados com stuffing, delimitador final

    // Alocar memória para o quadro final
    uint8_t *final_frame = (uint8_t *)malloc(frame_length);
    if (final_frame == NULL)
    {
        perror("Falha ao alocar memória para final_frame");
        return;
    }

    // Construir o quadro final concatenando todas as partes
    uint16_t pos = 0;
    final_frame[pos++] = FRAME_DELIMITER;                      // Delimitador inicial
    memcpy(&final_frame[pos], stuffed_buffer, stuffed_length);  // Dados com stuffing
    pos += stuffed_length;
    final_frame[pos++] = FRAME_DELIMITER;                      // Delimitador final

    // Enviar o quadro final via porta serial
    bytes_written = write(fd, final_frame, frame_length);
    if (bytes_written < 0)
    {
        perror("Erro ao escrever na porta serial");
    }
    else
    {
        pthread_mutex_lock(&print_mutex);  // Aciona bloqueio do mutex
        printf("Escreveu %d bytes na porta serial com sucesso.\n", bytes_written);
        pthread_mutex_unlock(&print_mutex);  // Aciona desbloqueio do mutex
    }

    // Liberar a memória alocada para o quadro final
    free(final_frame);

    // Ler a resposta
    // A resposta será lida pela thread assíncrona
}

// Função para aplicar byte stuffing
void byte_stuffing(uint8_t *input, uint16_t length, uint8_t *output, uint16_t *out_length)
{
    uint16_t i, j = 0;
    for (i = 0; i < length; i++)
    {
        if (input[i] == FRAME_DELIMITER || input[i] == ESCAPE_CHAR)
        {
            output[j++] = ESCAPE_CHAR;
            output[j++] = input[i] ^ XOR_VALUE;
        }
        else
        {
            output[j++] = input[i];
        }
    }
    *out_length = j;
}

// Função para remover byte stuffing
int byte_unstuffing(uint8_t *input, uint16_t length, uint8_t *output, uint16_t *out_length)
{
    uint16_t i = 0, j = 0;
    while (i < length)
    {
        if (input[i] == ESCAPE_CHAR)
        {
            i++;
            if (i >= length)
                return -1; // Erro: escape sem caractere seguinte
            output[j++] = input[i++] ^ XOR_VALUE; // Operação XOR
        }
        else
        {
            output[j++] = input[i++];
        }
    }
    *out_length = j;
    return 0;
}

// Função para gerar um TID aleatório
uint16_t generate_tid()
{
    return (uint16_t)(rand() % 65535 + 1);
}

void user_interface()
{
    pthread_mutex_lock(&print_mutex);  // Aciona bloqueio do mutex

    printf("\nComandos Disponíveis:\n");
    printf("1. Ler Entrada\n");
    printf("2. Ler Saída\n");
    printf("3. Escrever Saída\n");
    printf("4. Configurar Gatilho de Entrada\n");
    printf("5. Sair\n");
    printf("Digite o número da opção desejada: ");

    pthread_mutex_unlock(&print_mutex);  // Aciona desbloqueio do mutex

    int choice;
    int result = scanf("%d", &choice);

    // Limpar o buffer de entrada
    while (getchar() != '\n')
        ;


    if (result == 1)
    {
        Command cmd;
        uint16_t addr;
        uint16_t value;

        switch (choice)
        {
        case 1:
            // Ler Entrada
            pthread_mutex_lock(&print_mutex); // Aciona bloqueio do mutex
            printf("Entradas Disponíveis:\n");
            printf("1. SW1\n");
            printf("2. SW2\n");
            printf("3. SW3\n");
            printf("4. SW4\n");
            printf("Digite o número da entrada: ");
            pthread_mutex_unlock(&print_mutex); // Aciona desbloqueio do mutex

            scanf("%hu", &addr);
            while (getchar() != '\n')
                ;
            switch (addr)
            {
            case 1:
                cmd.addr = ADDR_SW1;
                break;
            case 2:
                cmd.addr = ADDR_SW2;
                break;
            case 3:
                cmd.addr = ADDR_SW3;
                break;
            case 4:
                cmd.addr = ADDR_SW4;
                break;
            default:
                pthread_mutex_lock(&print_mutex); // Aciona bloqueio do mutex
                printf("Endereço inválido.\n");
                pthread_mutex_unlock(&print_mutex); // Aciona desbloqueio do mutex
                return;
            }
            cmd.tid = generate_tid();
            cmd.oper = OP_READ;
            cmd.data = 0;
            send_command(fd, &cmd);
            break;

        case 2:
            // Ler Saída
            pthread_mutex_lock(&print_mutex); // Aciona bloqueio do mutex
            printf("Saídas Disponíveis:\n");
            printf("1. LED1\n");
            printf("2. LED2\n");
            printf("3. LED3\n");
            printf("4. LED4\n");
            printf("Digite o número da saída: ");
            pthread_mutex_unlock(&print_mutex); // Aciona desbloqueio do mutex

            scanf("%hu", &addr);
            while (getchar() != '\n')
                ;
            switch (addr)
            {
            case 1:
                cmd.addr = ADDR_LED1;
                break;
            case 2:
                cmd.addr = ADDR_LED2;
                break;
            case 3:
                cmd.addr = ADDR_LED3;
                break;
            case 4:
                cmd.addr = ADDR_LED4;
                break;
            default:
                pthread_mutex_lock(&print_mutex); // Aciona bloqueio do mutex
                printf("Endereço inválido.\n");
                pthread_mutex_unlock(&print_mutex); // Aciona desbloqueio do mutex
                return;
            }
            cmd.tid = generate_tid();
            cmd.oper = OP_READ;
            cmd.data = 0;
            send_command(fd, &cmd);
            break;

        case 3:
            // Escrever Saída
            pthread_mutex_lock(&print_mutex); // Aciona bloqueio do mutex
            printf("Saídas Disponíveis:\n");
            printf("1. LED1\n");
            printf("2. LED2\n");
            printf("3. LED3\n");
            printf("4. LED4\n");
            printf("Digite o número da saída: ");
            pthread_mutex_unlock(&print_mutex); // Aciona desbloqueio do mutex

            scanf("%hu", &addr);
            while (getchar() != '\n')
                ;
            switch (addr)
            {
            case 1:
                cmd.addr = ADDR_LED1;
                break;
            case 2:
                cmd.addr = ADDR_LED2;
                break;
            case 3:
                cmd.addr = ADDR_LED3;
                break;
            case 4:
                cmd.addr = ADDR_LED4;
                break;
            default:
                pthread_mutex_lock(&print_mutex); // Aciona bloqueio do mutex
                printf("Endereço inválido.\n");
                pthread_mutex_unlock(&print_mutex); // Aciona desbloqueio do mutex
                return;
            }
            pthread_mutex_lock(&print_mutex); // Aciona bloqueio do mutex
            printf("Digite o valor para a saída (0 ou 1): ");
            pthread_mutex_unlock(&print_mutex); // Aciona desbloqueio do mutex

            scanf("%hu", &value);
            while (getchar() != '\n')
                ;
            if (value != 0 && value != 1)
            {
                pthread_mutex_lock(&print_mutex); // Aciona bloqueio do mutex
                printf("Valor inválido.\n");
                pthread_mutex_unlock(&print_mutex); // Aciona desbloqueio do mutex
                return;
            }
            cmd.tid = generate_tid();
            cmd.oper = OP_WRITE;
            cmd.data = value;
            send_command(fd, &cmd);
            break;

        case 4:
            // Configurar Gatilho de Entrada
            pthread_mutex_lock(&print_mutex); // Aciona bloqueio do mutex
            printf("Entradas Disponíveis para Configurar Gatilho:\n");
            printf("1. SW1\n");
            printf("2. SW2\n");
            printf("3. SW3\n");
            printf("4. SW4\n");
            printf("Digite o número da entrada: ");
            pthread_mutex_unlock(&print_mutex); // Aciona desbloqueio do mutex

            scanf("%hu", &addr);
            while (getchar() != '\n')
                ;
            switch (addr)
            {
            case 1:
                cmd.addr = ADDR_SW1_TRIGGER;
                break;
            case 2:
                cmd.addr = ADDR_SW2_TRIGGER;
                break;
            case 3:
                cmd.addr = ADDR_SW3_TRIGGER;
                break;
            case 4:
                cmd.addr = ADDR_SW4_TRIGGER;
                break;
            default:
                pthread_mutex_lock(&print_mutex); // Aciona bloqueio do mutex
                printf("Endereço inválido.\n");
                pthread_mutex_unlock(&print_mutex); // Aciona desbloqueio do mutex
                return;
            }
            pthread_mutex_lock(&print_mutex); // Aciona bloqueio do mutex
            printf("Tipos de Gatilho Disponíveis:\n");
            printf("0. Desativar Gatilho\n");
            printf("1. Borda de Subida\n");
            printf("2. Borda de Descida\n");
            printf("3. Ambas as Bordas\n");
            printf("Digite o tipo de gatilho: ");
            pthread_mutex_unlock(&print_mutex); // Aciona desbloqueio do mutex

            scanf("%hu", &value);
            while (getchar() != '\n')
                ;
            if (value > 3)
            {
                pthread_mutex_lock(&print_mutex); // Aciona bloqueio do mutex
                printf("Valor inválido.\n");
                pthread_mutex_unlock(&print_mutex); // Aciona desbloqueio do mutex
                return;
            }
            cmd.tid = generate_tid();
            cmd.oper = OP_WRITE;
            cmd.data = value;
            send_command(fd, &cmd);
            break;

        case 5:
            pthread_mutex_lock(&print_mutex); // Aciona bloqueio do mutex
            printf("Encerrando a aplicação...\n");
            pthread_mutex_unlock(&print_mutex); // Aciona desbloqueio do mutex
            running = 0;
            break;

        default:
            pthread_mutex_lock(&print_mutex); // Aciona bloqueio do mutex
            printf("Opção inválida.\n");
            pthread_mutex_unlock(&print_mutex); // Aciona desbloqueio do mutex
            break;
        }
    }
    else
    {
        pthread_mutex_lock(&print_mutex); // Aciona bloqueio do mutex
        printf("Entrada inválida.\n");
        pthread_mutex_unlock(&print_mutex); // Aciona desbloqueio do mutex
    }
}

// Função para limpeza
void cleanup()
{
    // Fechar a porta serial
    close(fd);
}

void *asynchronous_reader(void *arg)
{
    uint8_t read_buffer[READ_BUF_SIZE];
    uint8_t frame_buffer[MAX_BUFFER_SIZE];
    uint16_t frame_index = 0;
    uint8_t in_frame = 0;
    int bytes_read;

    while (running)
    {
        bytes_read = read(fd, read_buffer, sizeof(read_buffer)); // Ler buffer da serial
        if (bytes_read > 0)
        {
            uint8_t *ptr = read_buffer;
            uint8_t *end = read_buffer + bytes_read;

            while (ptr < end)
            {
                uint8_t c = *ptr++;
                if (c == FRAME_DELIMITER) // Se 0x7E (delimitador)
                {
                    if (in_frame)
                    {
                        // Fim do quadro
                        uint8_t unstuffed_data[MAX_BUFFER_SIZE];
                        uint16_t unstuffed_length;
                        if (byte_unstuffing(frame_buffer, frame_index, unstuffed_data, &unstuffed_length) != 0)
                        {
                            fprintf(stderr, "Erro no byte unstuffing.\n");
                            frame_index = 0;
                            in_frame = 0;
                            continue;
                        }
                        process_received_data(unstuffed_data, unstuffed_length);
                        frame_index = 0;
                        in_frame = 0;
                    }
                    else
                    {
                        // Início do quadro
                        in_frame = 1;
                        frame_index = 0;
                    }
                }
                else if (in_frame)
                {
                    if (frame_index < MAX_BUFFER_SIZE)
                    {
                        frame_buffer[frame_index++] = c;
                    }
                    else
                    {
                        fprintf(stderr, "Overflow do buffer de quadro.\n");
                        frame_index = 0;
                        in_frame = 0;
                    }
                }
            }
        }
        else if (bytes_read < 0)
        {
            fprintf(stderr, "Erro ao ler da porta serial: %s\n", strerror(errno));
            usleep(100000); // Espera 100 ms
        }
        else
        {
            // Nenhum dado disponível, espera um pouco
            usleep(100000); // 100 ms
        }
    }

    return NULL;
}