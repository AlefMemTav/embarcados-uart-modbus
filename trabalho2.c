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
short calcula_CRC(unsigned char *commands, int size);
short CRC16(short crc, char data);
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
    
    uint16_t crc = calcula_CRC(frame_sem_crc, sizeof(frame_sem_crc));

    uint8_t tx_buffer[sizeof(frame_sem_crc) + 2];
    memcpy(tx_buffer, frame_sem_crc, sizeof(frame_sem_crc));
    tx_buffer[sizeof(frame_sem_crc)] = crc & 0xFF;
    tx_buffer[sizeof(frame_sem_crc) + 1] = (crc >> 8) & 0xFF;

    print_buffer_hex("--> Enviando Leitura", tx_buffer, sizeof(tx_buffer));
    if (write(uart_fd, tx_buffer, sizeof(tx_buffer)) <= 0) {
        perror("UART TX error");
        close(uart_fd);
        return;
    }

    // --- Leitura e Validação da Resposta ---
    uint8_t rx_buffer[256];
    int bytes_lidos = read(uart_fd, rx_buffer, 255);

    if (bytes_lidos > 0) {
        print_buffer_hex("<-- Resposta Recebida", rx_buffer, bytes_lidos);

        if (bytes_lidos < 5) { // Mínimo: [ADDR][FUNC][BYTE_COUNT][CRC1][CRC2]
            printf("Erro: Resposta Modbus muito curta.\n");
        } else {
            uint16_t crc_recebido = (rx_buffer[bytes_lidos - 1] << 8) | rx_buffer[bytes_lidos - 2];
            uint16_t crc_calculado = calcula_CRC(rx_buffer, bytes_lidos - 2);

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
        printf("Dados insuficientes para comandos X e Y.\n");
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

// *** NOVAS FUNÇÕES DE CRC (SUBSTITUINDO A ANTIGA) ***
/**
 * @brief Calcula um passo do CRC de 16 bits usando uma tabela de consulta.
 * @param crc O valor atual do CRC.
 * @param data O byte de dado a ser processado.
 * @return O novo valor do CRC.
 */
short CRC16(short crc, char data)
 {
     const short tbl[256] = {
         0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
         0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
         0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
         0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
         0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
         0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
         0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
         0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
         0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
         0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
         0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
         0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
         0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
         0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
         0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
         0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
         0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
         0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
         0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
         0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
         0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
         0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
         0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
         0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
         0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
         0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
         0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
         0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
         0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
         0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
         0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
         0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040};
     return ((crc & 0xFF00) >> 8) ^ tbl[(crc & 0x00FF) ^ (data & 0x00FF)];
 }
 
 /**
  * @brief Calcula o CRC-16 para um buffer de dados completo.
  * @param commands Ponteiro para o buffer de dados.
  * @param size O tamanho do buffer.
  * @return O valor final do CRC.
  */
 short calcula_CRC(unsigned char *commands, int size) {
     int i;
     short crc = 0xFFFF; // O CRC Modbus RTU deve ser inicializado com 0xFFFF
     for(i=0;i<size;i++) {
         crc = CRC16(crc, commands[i]);
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
