#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>      // Para usleep() e funções de arquivo POSIX
#include <fcntl.h>       // Para as flags de open()
#include <termios.h>     // Para configuração da UART
#include <errno.h>       // Para tratamento de erros
#include <stdint.h>      // Para tipos como uint16_t e uint8_t

// --- Constantes do Protocolo Modbus ---
#define ESP32_ADDRESS           0x01
#define FUNC_READ_REGS          0x03
#define FUNC_WRITE_REG          0x06

// --- Endereços dos Registradores de Comando na ESP32 ---
#define REG_ADDR_CMD_X          0x0000
#define REG_ADDR_CMD_Y          0x0001

// --- Máscaras de Bits para Comandos ---
#define CMD_X_ESQUERDA          0x01
#define CMD_X_DIREITA           0x02
#define CMD_Y_CIMA              0x01
#define CMD_Y_BAIXO             0x02

// --- Configurações Gerais ---
const uint8_t MATRICULA[4] = {5, 5, 4, 8}; // Matrícula como valores numéricos
const char* UART_DEVICE = "/dev/serial0";

// --- Protótipos das Funções ---
int abrir_e_configurar_uart();
uint16_t crc16_modbus(const uint8_t *data, uint16_t length);
void print_buffer_hex(const char* label, const uint8_t* buffer, int length);
void poll_command_registers();
void parse_and_execute_commands(const uint8_t* data, int data_len);

/**
 * @brief Função principal. Executa um loop infinito para ler os comandos da ESP32.
 */
int main() {
    printf("Iniciando controle da máquina via Modbus RTU...\n");
    printf("Pressione CTRL+C para sair.\n");

    while (1) {
        poll_command_registers();
        usleep(50000); // Aguarda 50 ms antes da próxima leitura
    }

    return 0;
}

/**
 * @brief Abre e configura a porta UART com os novos parâmetros (115200 bps).
 * @return File descriptor da porta ou -1 em caso de erro.
 */
int abrir_e_configurar_uart() {
    int uart_fd = open(UART_DEVICE, O_RDWR | O_NOCTTY);
    if (uart_fd == -1) {
        perror("Erro ao abrir UART");
        return -1;
    }

    struct termios options;
    tcgetattr(uart_fd, &options);

    // Configuração da UART: 115200 bps, 8N1
    cfsetospeed(&options, B115200);
    cfsetispeed(&options, B115200);
    options.c_cflag &= ~PARENB; // Sem paridade
    options.c_cflag &= ~CSTOPB; // 1 stop bit
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;      // 8 data bits
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 10; // Timeout de 1 segundo para leitura

    tcflush(uart_fd, TCIFLUSH);
    tcsetattr(uart_fd, TCSANOW, &options);

    return uart_fd;
}

/**
 * @brief Constrói e envia uma solicitação Modbus para ler os registradores de
 * comando da ESP32 e processa a resposta.
 */
void poll_command_registers() {
    int uart_fd = abrir_e_configurar_uart();
    if (uart_fd == -1) return;

    // Frame de requisição (Função 0x03): Ler 2 registradores a partir do endereço 0x0000
    // Payload customizado: [START_ADDR_HI] [START_ADDR_LO] [QTD_REGS_HI] [QTD_REGS_LO] [MATRICULA]
    uint8_t payload[2 + 2 + 4];
    payload[0] = (REG_ADDR_CMD_X >> 8) & 0xFF; // Endereço inicial (High)
    payload[1] = REG_ADDR_CMD_X & 0xFF;       // Endereço inicial (Low)
    payload[2] = 0x00;                        // Quantidade de regs (High)
    payload[3] = 0x02;                        // Quantidade de regs (Low) -> Ler 2 registradores
    memcpy(&payload[4], MATRICULA, 4);

    uint8_t frame_sem_crc[1 + 1 + sizeof(payload)];
    frame_sem_crc[0] = ESP32_ADDRESS;
    frame_sem_crc[1] = FUNC_READ_REGS;
    memcpy(&frame_sem_crc[2], payload, sizeof(payload));
    
    uint16_t crc = crc16_modbus(frame_sem_crc, sizeof(frame_sem_crc));

    uint8_t tx_buffer[sizeof(frame_sem_crc) + 2];
    memcpy(tx_buffer, frame_sem_crc, sizeof(frame_sem_crc));
    tx_buffer[sizeof(frame_sem_crc)] = crc & 0xFF;
    tx_buffer[sizeof(frame_sem_crc) + 1] = (crc >> 8) & 0xFF;

    // print_buffer_hex("--> Enviando Leitura", tx_buffer, sizeof(tx_buffer));
    if (write(uart_fd, tx_buffer, sizeof(tx_buffer)) <= 0) {
        perror("UART TX error");
        close(uart_fd);
        return;
    }

    // --- Leitura e Validação da Resposta ---
    uint8_t rx_buffer[256];
    int bytes_lidos = read(uart_fd, rx_buffer, 255);

    if (bytes_lidos > 0) {
        // print_buffer_hex("<-- Resposta Recebida", rx_buffer, bytes_lidos);

        if (bytes_lidos < 5) { // Mínimo: [ADDR][FUNC][BYTE_COUNT][CRC1][CRC2]
            printf("Erro: Resposta Modbus muito curta.\n");
        } else {
            uint16_t crc_recebido = (rx_buffer[bytes_lidos - 1] << 8) | rx_buffer[bytes_lidos - 2];
            uint16_t crc_calculado = crc16_modbus(rx_buffer, bytes_lidos - 2);

            if (crc_calculado != crc_recebido) {
                printf("ERRO DE CRC! Mensagem descartada.\n");
            } else if (rx_buffer[0] == ESP32_ADDRESS && rx_buffer[1] == FUNC_READ_REGS) {
                // Resposta válida, processa os dados
                uint8_t byte_count = rx_buffer[2];
                parse_and_execute_commands(&rx_buffer[3], byte_count);
            } else if (rx_buffer[1] == (FUNC_READ_REGS | 0x80)) {
                printf("ESP32 retornou um erro Modbus! Código de exceção: 0x%02X\n", rx_buffer[2]);
            }
        }
    }
    close(uart_fd);
}

/**
 * @brief Interpreta os dados recebidos da ESP32 e imprime a ação correspondente.
 * @param data Ponteiro para o início dos dados no buffer de resposta.
 * @param data_len Quantidade de bytes de dados.
 */
void parse_and_execute_commands(const uint8_t* data, int data_len) {
    if (data_len < 2) {
        // printf("Dados insuficientes para comandos X e Y.\n");
        return;
    }

    uint8_t cmd_x = data[0]; // Dados do registrador 0x00
    uint8_t cmd_y = data[1]; // Dados do registrador 0x01

    // Processa Eixo X
    if (cmd_x == CMD_X_DIREITA) {
        printf("Comando: Mover para a DIREITA [>>]\n");
    } else if (cmd_x == CMD_X_ESQUERDA) {
        printf("Comando: Mover para a ESQUERDA [<<]\n");
    }

    // Processa Eixo Y
    if (cmd_y == CMD_Y_BAIXO) {
        printf("Comando: Mover para BAIXO [\\/]\n");
    } else if (cmd_y == CMD_Y_CIMA) {
        printf("Comando: Mover para CIMA [/\\\\]\n");
    }
}

// --- Funções Auxiliares (mantidas para completude e depuração) ---

/**
 * @brief Calcula o CRC-16 para um buffer de dados.
 */
uint16_t crc16_modbus(const uint8_t *data, uint16_t length) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc = crc >> 1;
            }
        }
    }
    return crc;
}

/**
 * @brief Imprime o conteúdo de um buffer em formato hexadecimal.
 */
void print_buffer_hex(const char* label, const uint8_t* buffer, int length) {
    printf("%s (%d bytes): ", label, length);
    for (int i = 0; i < length; i++) {
        printf("0x%02X ", buffer[i]);
    }
    printf("\n");
}
