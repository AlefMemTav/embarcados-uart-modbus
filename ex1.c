#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>      // Para usleep() e funções de arquivo POSIX (read, write, close)
#include <fcntl.h>       // Para as flags de open() como O_RDWR
#include <termios.h>     // Para configuração da UART (struct termios)
#include <errno.h>       // Para tratamento de erros

// --- Constantes do Protocolo ---
#define CMD_SOLICITA_INT    0xA1
#define CMD_SOLICITA_FLOAT  0xA2
#define CMD_SOLICITA_STRING 0xA3
#define CMD_ENVIA_INT       0xB1
#define CMD_ENVIA_FLOAT     0xB2
#define CMD_ENVIA_STRING    0xB3

// Defina os 4 últimos dígitos da sua matrícula aqui
const char MATRICULA[4] = {'5', '5', '4', '8'};
const char* UART_DEVICE = "/dev/serial0";

// --- Protótipos das Funções ---
void mostrar_menu();
int abrir_e_configurar_uart();
void solicitar_dado(unsigned char codigo_comando);
void enviar_inteiro();
void enviar_float();
void enviar_string();

/**
 * @brief Função principal que exibe o menu e gerencia a seleção do usuário.
 */
int main() {
    int escolha = 0;
    do {
        mostrar_menu();
        printf("Sua escolha: ");
        scanf("%d", &escolha);
        // Limpa o buffer de entrada para a próxima leitura
        while (getchar() != '\n'); 

        switch (escolha) {
            case 1:
                solicitar_dado(CMD_SOLICITA_INT);
                break;
            case 2:
                solicitar_dado(CMD_SOLICITA_FLOAT);
                break;
            case 3:
                solicitar_dado(CMD_SOLICITA_STRING);
                break;
            case 4:
                enviar_inteiro();
                break;
            case 5:
                enviar_float();
                break;
            case 6:
                enviar_string();
                break;
            case 0:
                printf("Saindo do programa.\n");
                break;
            default:
                printf("Opção inválida! Tente novamente.\n");
        }
        printf("\n----------------------------------------\n");
    } while (escolha != 0);

    return 0;
}

/**
 * @brief Exibe o menu de opções para o usuário.
 */
void mostrar_menu() {
    printf("\n### MENU DE COMUNICAÇÃO UART RASPBERRY PI ###\n");
    printf("--- Solicitar Dados do Arduino ---\n");
    printf("1. Solicitar Inteiro (0xA1)\n");
    printf("2. Solicitar Float   (0xA2)\n");
    printf("3. Solicitar String  (0xA3)\n");
    printf("--- Enviar Dados para o Arduino ---\n");
    printf("4. Enviar Inteiro (0xB1)\n");
    printf("5. Enviar Float   (0xB2)\n");
    printf("6. Enviar String  (0xB3)\n");
    printf("----------------------------------------\n");
    printf("0. Sair\n");
}

/**
 * @brief Abre a porta serial e a configura com os parâmetros desejados.
 * @return Retorna o file descriptor da porta ou -1 em caso de erro.
 */
int abrir_e_configurar_uart() {
    int uart_fd = open(UART_DEVICE, O_RDWR | O_NOCTTY | O_NDELAY);
    if (uart_fd == -1) {
        perror("Erro - Não foi possível iniciar a UART");
        return -1;
    }

    struct termios options;
    tcgetattr(uart_fd, &options);
    options.c_cflag = B9600 | CS8 | CLOCAL | CREAD; // Baud rate 9600, 8 bits, local, habilitar leitura
    options.c_iflag = IGNPAR; // Ignorar erros de paridade
    options.c_oflag = 0;
    options.c_lflag = 0;
    tcflush(uart_fd, TCIFLUSH); // Descarta dados que não foram lidos
    tcsetattr(uart_fd, TCSANOW, &options);

    return uart_fd;
}

/**
 * @brief Envia uma solicitação de dados para o Arduino e lê a resposta.
 * @param codigo_comando O código do comando de solicitação (0xA1, 0xA2, 0xA3).
 */
void solicitar_dado(unsigned char codigo_comando) {
    int uart_fd = abrir_e_configurar_uart();
    if (uart_fd == -1) return;

    // Monta a mensagem de solicitação: [COMANDO] + [MATRICULA]
    unsigned char tx_buffer[5];
    tx_buffer[0] = codigo_comando;
    memcpy(&tx_buffer[1], MATRICULA, 4);

    printf("Enviando solicitação com código: 0x%02X\n", codigo_comando);
    if (write(uart_fd, tx_buffer, 5) <= 0) {
        perror("UART TX error");
        close(uart_fd);
        return;
    }

    // Aguarda a resposta do Arduino
    usleep(100000); // 100 ms

    unsigned char rx_buffer[256];
    int bytes_lidos = 0;

    // Tenta ler a resposta do buffer da UART
    bytes_lidos = read(uart_fd, rx_buffer, 255);

    if (bytes_lidos <= 0) {
        printf("Nenhum dado recebido do dispositivo.\n");
    } else {
        printf("%d bytes recebidos.\n", bytes_lidos);
        switch (codigo_comando) {
            case CMD_SOLICITA_INT: {
                if(bytes_lidos == 4) {
                    int valor_int = *((int*)rx_buffer);
                    printf(">> DADO INT RECEBIDO: %d\n", valor_int);
                } else {
                    printf("Erro: Esperava 4 bytes para int, mas recebi %d.\n", bytes_lidos);
                }
                break;
            }
            case CMD_SOLICITA_FLOAT: {
                 if(bytes_lidos == 4) {
                    float valor_float = *((float*)rx_buffer);
                    printf(">> DADO FLOAT RECEBIDO: %f\n", valor_float);
                } else {
                    printf("Erro: Esperava 4 bytes para float, mas recebi %d.\n", bytes_lidos);
                }
                break;
            }
            case CMD_SOLICITA_STRING: {
                unsigned char tamanho_string = rx_buffer[0];
                // Verifica se a mensagem está completa
                if (bytes_lidos == tamanho_string + 1) {
                    char str_conteudo[256];
                    memcpy(str_conteudo, &rx_buffer[1], tamanho_string);
                    str_conteudo[tamanho_string] = '\0'; // Null-terminator
                    printf(">> DADO STRING RECEBIDO (%d chars): %s\n", tamanho_string, str_conteudo);
                } else {
                    printf("Erro: Pacote de string incompleto. Esperava %d bytes, recebi %d\n", tamanho_string + 1, bytes_lidos);
                }
                break;
            }
        }
    }
    close(uart_fd);
}


/**
 * @brief Solicita um inteiro ao usuário, envia para o Arduino e imprime o retorno.
 */
void enviar_inteiro() {
    int uart_fd = abrir_e_configurar_uart();
    if (uart_fd == -1) return;

    int valor_int;
    printf("Digite o valor inteiro a ser enviado: ");
    scanf("%d", &valor_int);

    // Monta a mensagem: [COMANDO] + [DADO_INT] + [MATRICULA]
    unsigned char tx_buffer[1 + 4 + 4];
    tx_buffer[0] = CMD_ENVIA_INT;
    memcpy(&tx_buffer[1], &valor_int, 4);
    memcpy(&tx_buffer[5], MATRICULA, 4);

    printf("Enviando inteiro: %d\n", valor_int);
    if (write(uart_fd, tx_buffer, sizeof(tx_buffer)) <= 0) {
        perror("UART TX error");
        close(uart_fd);
        return;
    }
    
    usleep(100000); // Aguarda resposta

    int retorno_int;
    if (read(uart_fd, &retorno_int, 4) == 4) {
        printf(">> Arduino retornou o inteiro: %d\n", retorno_int);
    } else {
        printf("Nenhuma confirmação recebida.\n");
    }

    close(uart_fd);
}

/**
 * @brief Solicita um float ao usuário, envia para o Arduino e imprime o retorno.
 */
void enviar_float() {
    int uart_fd = abrir_e_configurar_uart();
    if (uart_fd == -1) return;

    float valor_float;
    printf("Digite o valor float a ser enviado: ");
    scanf("%f", &valor_float);

    // Monta a mensagem: [COMANDO] + [DADO_FLOAT] + [MATRICULA]
    unsigned char tx_buffer[1 + 4 + 4];
    tx_buffer[0] = CMD_ENVIA_FLOAT;
    memcpy(&tx_buffer[1], &valor_float, 4);
    memcpy(&tx_buffer[5], MATRICULA, 4);

    printf("Enviando float: %f\n", valor_float);
    if (write(uart_fd, tx_buffer, sizeof(tx_buffer)) <= 0) {
        perror("UART TX error");
        close(uart_fd);
        return;
    }

    usleep(100000); // Aguarda resposta

    float retorno_float;
    if (read(uart_fd, &retorno_float, 4) == 4) {
        printf(">> Arduino retornou o float: %f\n", retorno_float);
    } else {
        printf("Nenhuma confirmação recebida.\n");
    }

    close(uart_fd);
}

/**
 * @brief Solicita uma string ao usuário, envia para o Arduino e imprime o retorno.
 */
void enviar_string() {
    int uart_fd = abrir_e_configurar_uart();
    if (uart_fd == -1) return;

    char str_envio[250];
    printf("Digite a string a ser enviada (max 249 chars): ");
    // Limpa o buffer de entrada antes de ler a string
    while (getchar() != '\n'); 
    fgets(str_envio, sizeof(str_envio), stdin);
    str_envio[strcspn(str_envio, "\n")] = 0; // Remove o '\n' do final

    unsigned char tamanho = (unsigned char)strlen(str_envio);

    // Monta a mensagem: [COMANDO] + [TAMANHO] + [STRING] + [MATRICULA]
    int tamanho_total = 1 + 1 + tamanho + 4;
    unsigned char tx_buffer[tamanho_total];
    tx_buffer[0] = CMD_ENVIA_STRING;
    tx_buffer[1] = tamanho;
    memcpy(&tx_buffer[2], str_envio, tamanho);
    memcpy(&tx_buffer[2 + tamanho], MATRICULA, 4);

    printf("Enviando string: '%s' (tamanho %d)\n", str_envio, tamanho);
    if (write(uart_fd, tx_buffer, tamanho_total) <= 0) {
        perror("UART TX error");
        close(uart_fd);
        return;
    }

    usleep(100000); // Aguarda resposta

    // Lê a resposta (tamanho + string)
    unsigned char rx_buffer[256];
    int bytes_lidos = read(uart_fd, rx_buffer, 255);
    if (bytes_lidos > 0) {
        unsigned char tamanho_retorno = rx_buffer[0];
        if (bytes_lidos == tamanho_retorno + 1) {
             char str_retorno[256];
             memcpy(str_retorno, &rx_buffer[1], tamanho_retorno);
             str_retorno[tamanho_retorno] = '\0';
             printf(">> Arduino retornou a string (%d chars): %s\n", tamanho_retorno, str_retorno);
        } else {
             printf("Erro: Pacote de retorno de string incompleto.\n");
        }
    } else {
        printf("Nenhuma confirmação recebida.\n");
    }

    close(uart_fd);
}
