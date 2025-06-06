#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>      // Para usleep() e funções de arquivo POSIX
#include <fcntl.h>       // Para as flags de open()
#include <termios.h>     // Para configuração da UART
#include <errno.h>       // Para tratamento de erros
#include <stdint.h>      // Para tipos como uint16_t e uint8_t

// --- Constantes do Protocolo Modbus-Like ---
#define DEVICE_ADDRESS          0x01
#define FUNC_SOLICITAR_DADOS    0x23
#define FUNC_ENVIAR_DADOS       0x16

// Sub-códigos
#define SUB_CMD_INT             0xA1 // Usado para solicitar e como prefixo para enviar
#define SUB_CMD_FLOAT           0xA2
#define SUB_CMD_STRING          0xA3
#define SUB_CMD_WRITE_INT       0xB1 // Comandos da tabela original para envio
#define SUB_CMD_WRITE_FLOAT     0xB2
#define SUB_CMD_WRITE_STRING    0xB3


// Defina os 4 últimos dígitos da sua matrícula como valores numéricos
const uint8_t MATRICULA[4] = {5, 5, 4, 8};
const char* UART_DEVICE = "/dev/serial0";

// --- Protótipos das Funções ---
void mostrar_menu();
int abrir_e_configurar_uart();
uint16_t crc16_modbus(const uint8_t *data, uint16_t length);
void print_buffer_hex(const char* label, const uint8_t* buffer, int length);

void solicitar_dado_modbus(uint8_t sub_codigo);
void enviar_inteiro_modbus();
void enviar_float_modbus();
void enviar_string_modbus();

/**
 * @brief Função principal que exibe o menu e gerencia a seleção do usuário.
 */
int main() {
    int escolha = 0;
    do {
        mostrar_menu();
        printf("Sua escolha: ");
        if (scanf("%d", &escolha) != 1) {
            escolha = -1; // Força erro se a entrada não for um número
        }
        // Limpa o buffer de entrada para a próxima leitura
        while (getchar() != '\n'); 

        switch (escolha) {
            case 1: solicitar_dado_modbus(SUB_CMD_INT); break;
            case 2: solicitar_dado_modbus(SUB_CMD_FLOAT); break;
            case 3: solicitar_dado_modbus(SUB_CMD_STRING); break;
            case 4: enviar_inteiro_modbus(); break;
            case 5: enviar_float_modbus(); break;
            case 6: enviar_string_modbus(); break;
            case 0: printf("Saindo do programa.\n"); break;
            default: printf("Opção inválida! Tente novamente.\n"); break;
        }
        printf("\n--------------------------------------------------\n");
    } while (escolha != 0);

    return 0;
}

/**
 * @brief Exibe o menu de opções para o usuário.
 */
void mostrar_menu() {
    printf("\n### MENU DE COMUNICAÇÃO UART (MODBUS-LIKE) ###\n");
    printf("--- Solicitar Dados do Arduino (Função 0x23) ---\n");
    printf("1. Solicitar Inteiro (Sub-código 0xA1)\n");
    printf("2. Solicitar Float   (Sub-código 0xA2)\n");
    printf("3. Solicitar String  (Sub-código 0xA3)\n");
    printf("--- Enviar Dados para o Arduino (Função 0x16) ---\n");
    printf("4. Enviar Inteiro (Sub-código 0xB1)\n");
    printf("5. Enviar Float   (Sub-código 0xB2)\n");
    printf("6. Enviar String  (Sub-código 0xB3)\n");
    printf("--------------------------------------------------\n");
    printf("0. Sair\n");
}


/**
 * @brief Calcula o CRC-16 para um buffer de dados usando o polinômio do Modbus.
 * @param data Ponteiro para os dados.
 * @param length Comprimento dos dados em bytes.
 * @return O valor do CRC-16 calculado.
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
 * @param label Um rótulo para a impressão.
 * @param buffer O buffer a ser impresso.
 * @param length O tamanho do buffer.
 */
void print_buffer_hex(const char* label, const uint8_t* buffer, int length) {
    printf("%s (%d bytes): ", label, length);
    for (int i = 0; i < length; i++) {
        printf("0x%02X ", buffer[i]);
    }
    printf("\n");
}


/**
 * @brief Abre a porta serial e a configura com os parâmetros desejados.
 * @return Retorna o file descriptor da porta ou -1 em caso de erro.
 */
int abrir_e_configurar_uart() {
    int uart_fd = open(UART_DEVICE, O_RDWR | O_NOCTTY | O_NDELAY);
    if (uart_fd == -1) {
        perror("Erro ao abrir UART");
        return -1;
    }

    struct termios options;
    tcgetattr(uart_fd, &options);
    options.c_cflag = B9600 | CS8 | CLOCAL | CREAD;
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    tcflush(uart_fd, TCIFLUSH);
    tcsetattr(uart_fd, TCSANOW, &options);
    return uart_fd;
}

/**
 * @brief Envia uma solicitação de dados para o Arduino e processa a resposta.
 * @param sub_codigo O sub-código da solicitação (0xA1, 0xA2, 0xA3).
 */
void solicitar_dado_modbus(uint8_t sub_codigo) {
    int uart_fd = abrir_e_configurar_uart();
    if (uart_fd == -1) return;

    // Monta o frame: [ADDR] [FUNC] [SUB_CMD] [MATRICULA]
    uint8_t frame_sem_crc[1 + 1 + 1 + 4];
    frame_sem_crc[0] = DEVICE_ADDRESS;
    frame_sem_crc[1] = FUNC_SOLICITAR_DADOS;
    frame_sem_crc[2] = sub_codigo;
    memcpy(&frame_sem_crc[3], MATRICULA, 4);

    uint16_t crc = crc16_modbus(frame_sem_crc, sizeof(frame_sem_crc));

    uint8_t tx_buffer[sizeof(frame_sem_crc) + 2];
    memcpy(tx_buffer, frame_sem_crc, sizeof(frame_sem_crc));
    tx_buffer[sizeof(frame_sem_crc)] = crc & 0xFF;         // Low byte
    tx_buffer[sizeof(frame_sem_crc) + 1] = (crc >> 8) & 0xFF; // High byte

    print_buffer_hex("Enviando Frame de Solicitação", tx_buffer, sizeof(tx_buffer));
    if (write(uart_fd, tx_buffer, sizeof(tx_buffer)) <= 0) {
        perror("UART TX error");
        close(uart_fd);
        return;
    }

    usleep(200000); // Aguarda resposta (200ms)

    uint8_t rx_buffer[256];
    int bytes_lidos = read(uart_fd, rx_buffer, 255);

    if (bytes_lidos <= 0) {
        printf("Nenhuma resposta recebida do dispositivo.\n");
    } else {
        print_buffer_hex("Resposta Recebida", rx_buffer, bytes_lidos);

        // Validar CRC da resposta
        if (bytes_lidos < 4) {
             printf("Erro: Resposta muito curta para ser válida.\n");
        } else {
            uint16_t crc_recebido = (rx_buffer[bytes_lidos - 1] << 8) | rx_buffer[bytes_lidos - 2];
            uint16_t crc_calculado = crc16_modbus(rx_buffer, bytes_lidos - 2);

            if (crc_calculado != crc_recebido) {
                printf("ERRO DE CRC! Calculado: 0x%04X, Recebido: 0x%04X\n", crc_calculado, crc_recebido);
            } else {
                 printf("CRC OK.\n");
                // Decodifica a resposta se o CRC estiver correto
                // Frame de resposta esperado: [ADDR] [FUNC] [DATA...] [CRC]
                uint8_t* data_payload = &rx_buffer[2];
                int payload_len = bytes_lidos - 4; // Desconta ADDR, FUNC, e CRC(2)

                if (rx_buffer[1] == (FUNC_SOLICITAR_DADOS | 0x80)) {
                    printf("Erro Modbus recebido! Código de exceção: 0x%02X\n", data_payload[0]);
                } else if (rx_buffer[1] == FUNC_SOLICITAR_DADOS) {
                    switch (sub_codigo) {
                        case SUB_CMD_INT:
                            if(payload_len == 4) {
                                int32_t valor_int = *((int32_t*)data_payload);
                                printf(">> DADO INT RECEBIDO: %d\n", valor_int);
                            } else {
                                printf("Erro de formato: payload de inteiro com %d bytes.\n", payload_len);
                            }
                            break;
                        case SUB_CMD_FLOAT:
                            if(payload_len == 4) {
                                float valor_float = *((float*)data_payload);
                                printf(">> DADO FLOAT RECEBIDO: %f\n", valor_float);
                            } else {
                                printf("Erro de formato: payload de float com %d bytes.\n", payload_len);
                            }
                            break;
                        case SUB_CMD_STRING:
                            {
                                uint8_t str_len = data_payload[0];
                                if (payload_len == str_len + 1) {
                                    char str_conteudo[256];
                                    memcpy(str_conteudo, &data_payload[1], str_len);
                                    str_conteudo[str_len] = '\0';
                                    printf(">> DADO STRING RECEBIDO (%d chars): %s\n", str_len, str_conteudo);
                                } else {
                                     printf("Erro de formato: payload de string inconsistente.\n");
                                }
                            }
                            break;
                    }
                }
            }
        }
    }
    close(uart_fd);
}

/**
 * @brief Envia um inteiro para o Arduino e processa a confirmação.
 */
void enviar_inteiro_modbus() {
    int uart_fd = abrir_e_configurar_uart();
    if (uart_fd == -1) return;

    int32_t valor_int;
    printf("Digite o valor inteiro a ser enviado: ");
    scanf("%d", &valor_int);

    // Monta o payload: [SUB_CMD] [DADO_INT] [MATRICULA]
    uint8_t payload[1 + 4 + 4];
    payload[0] = SUB_CMD_WRITE_INT;
    memcpy(&payload[1], &valor_int, 4);
    memcpy(&payload[5], MATRICULA, 4);

    // Monta o frame: [ADDR] [FUNC] [PAYLOAD]
    uint8_t frame_sem_crc[1 + 1 + sizeof(payload)];
    frame_sem_crc[0] = DEVICE_ADDRESS;
    frame_sem_crc[1] = FUNC_ENVIAR_DADOS;
    memcpy(&frame_sem_crc[2], payload, sizeof(payload));

    uint16_t crc = crc16_modbus(frame_sem_crc, sizeof(frame_sem_crc));

    uint8_t tx_buffer[sizeof(frame_sem_crc) + 2];
    memcpy(tx_buffer, frame_sem_crc, sizeof(frame_sem_crc));
    tx_buffer[sizeof(frame_sem_crc)] = crc & 0xFF;
    tx_buffer[sizeof(frame_sem_crc) + 1] = (crc >> 8) & 0xFF;

    print_buffer_hex("Enviando Frame de Escrita", tx_buffer, sizeof(tx_buffer));
    write(uart_fd, tx_buffer, sizeof(tx_buffer));
    
    // Ler e validar a resposta de eco do Arduino
    usleep(200000);
    // (A lógica de leitura e validação da resposta seria similar à da função 'solicitar_dado_modbus')
    // Por simplicidade, vamos apenas imprimir o que foi recebido.
    uint8_t rx_buffer[256];
    int bytes_lidos = read(uart_fd, rx_buffer, 255);
     if (bytes_lidos > 0) {
        print_buffer_hex("Resposta Recebida (Eco)", rx_buffer, bytes_lidos);
    } else {
        printf("Nenhuma confirmação recebida.\n");
    }

    close(uart_fd);
}

/**
 * @brief Envia um float para o Arduino e processa a confirmação.
 */
void enviar_float_modbus() {
    int uart_fd = abrir_e_configurar_uart();
    if (uart_fd == -1) return;

    float valor_float;
    printf("Digite o valor float a ser enviado: ");
    scanf("%f", &valor_float);

    uint8_t payload[1 + 4 + 4];
    payload[0] = SUB_CMD_WRITE_FLOAT;
    memcpy(&payload[1], &valor_float, 4);
    memcpy(&payload[5], MATRICULA, 4);

    uint8_t frame_sem_crc[1 + 1 + sizeof(payload)];
    frame_sem_crc[0] = DEVICE_ADDRESS;
    frame_sem_crc[1] = FUNC_ENVIAR_DADOS;
    memcpy(&frame_sem_crc[2], payload, sizeof(payload));

    uint16_t crc = crc16_modbus(frame_sem_crc, sizeof(frame_sem_crc));
    uint8_t tx_buffer[sizeof(frame_sem_crc) + 2];
    memcpy(tx_buffer, frame_sem_crc, sizeof(frame_sem_crc));
    tx_buffer[sizeof(frame_sem_crc)] = crc & 0xFF;
    tx_buffer[sizeof(frame_sem_crc) + 1] = (crc >> 8) & 0xFF;

    print_buffer_hex("Enviando Frame de Escrita", tx_buffer, sizeof(tx_buffer));
    write(uart_fd, tx_buffer, sizeof(tx_buffer));

    usleep(200000);
    uint8_t rx_buffer[256];
    int bytes_lidos = read(uart_fd, rx_buffer, 255);
    if (bytes_lidos > 0) {
        print_buffer_hex("Resposta Recebida (Eco)", rx_buffer, bytes_lidos);
    } else {
        printf("Nenhuma confirmação recebida.\n");
    }

    close(uart_fd);
}

/**
 * @brief Envia uma string para o Arduino e processa a confirmação.
 */
void enviar_string_modbus() {
    int uart_fd = abrir_e_configurar_uart();
    if (uart_fd == -1) return;

    char str_envio[240];
    printf("Digite a string a ser enviada (max 239 chars): ");
    // Limpa o buffer de entrada antes de ler a string
    while (getchar() != '\n'); 
    fgets(str_envio, sizeof(str_envio), stdin);
    str_envio[strcspn(str_envio, "\n")] = 0; // Remove o '\n' do final

    uint8_t str_len = (uint8_t)strlen(str_envio);

    // Payload: [SUB_CMD] [STR_LEN] [STRING] [MATRICULA]
    uint16_t payload_len = 1 + 1 + str_len + 4;
    uint8_t payload[payload_len];
    payload[0] = SUB_CMD_WRITE_STRING;
    payload[1] = str_len;
    memcpy(&payload[2], str_envio, str_len);
    memcpy(&payload[2 + str_len], MATRICULA, 4);

    // Frame: [ADDR] [FUNC] [PAYLOAD]
    uint16_t frame_len_sem_crc = 1 + 1 + payload_len;
    uint8_t frame_sem_crc[frame_len_sem_crc];
    frame_sem_crc[0] = DEVICE_ADDRESS;
    frame_sem_crc[1] = FUNC_ENVIAR_DADOS;
    memcpy(&frame_sem_crc[2], payload, payload_len);

    uint16_t crc = crc16_modbus(frame_sem_crc, frame_len_sem_crc);

    uint8_t tx_buffer[frame_len_sem_crc + 2];
    memcpy(tx_buffer, frame_sem_crc, frame_len_sem_crc);
    tx_buffer[frame_len_sem_crc] = crc & 0xFF;
    tx_buffer[frame_len_sem_crc + 1] = (crc >> 8) & 0xFF;
    
    print_buffer_hex("Enviando Frame de Escrita", tx_buffer, sizeof(tx_buffer));
    write(uart_fd, tx_buffer, sizeof(tx_buffer));

    usleep(200000);
    uint8_t rx_buffer[256];
    int bytes_lidos = read(uart_fd, rx_buffer, 255);
    if (bytes_lidos > 0) {
        print_buffer_hex("Resposta Recebida (Eco)", rx_buffer, bytes_lidos);
    } else {
        printf("Nenhuma confirmação recebida.\n");
    }

    close(uart_fd);
}
