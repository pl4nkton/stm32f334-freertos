#include "term_rs232.h"
#include "command.h"
#include "stm32f30x.h"
#include "FreeRTOS.h"
#include "queue.h"
#include <stdio.h>
#include <stdlib.h>

#define RS232_BAUDRATE  115200
// #define RS232_BAUDRATE  230400
// #define RS232_BAUDRATE  460800
// #define RS232_BAUDRATE  921600

#define RX_QUEUE_SIZE  512
#define TX_QUEUE_SIZE  512


static volatile struct rs232_stats {
    uint32_t    rx_bytes;
    uint32_t    tx_bytes;
    uint32_t    rx_overrun;
    uint32_t    tx_overrun;
} rs232_stats;


static QueueHandle_t rx_queue = NULL;
static QueueHandle_t tx_queue = NULL;


void USART2_IRQHandler(void)
{
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    if (USART2->ISR & USART_ISR_RXNE) {
        char c = USART2->RDR;

        if (xQueueSendFromISR(rx_queue, &c, &xHigherPriorityTaskWoken))
            rs232_stats.rx_bytes++;
        else
            rs232_stats.rx_overrun++;
    }

    if (USART2->ISR & USART_ISR_TXE) {
        char c;
        if (xQueueReceiveFromISR(tx_queue, &c, &xHigherPriorityTaskWoken)) {
            // send a queued byte
            //
            USART2->TDR = c;
            rs232_stats.tx_bytes++;
        }
        else
            // nothing to send, disable interrupt
            //
            USART2->CR1 &= ~USART_CR1_TXEIE;
    }

    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}


static ssize_t rs232_write_r(struct _reent *r, int fd, const void *ptr, size_t len)
{
    const char *src = ptr;
    int sent = 0;

    while (len--) {

        // Convert c-newlines to terminal CRLF
        //
        if (*src == '\n') {
            xQueueSend(tx_queue, "\r", portMAX_DELAY);
            USART2->CR1 |= USART_CR1_TXEIE;
        }

        xQueueSend(tx_queue, src++, portMAX_DELAY);

        USART2->CR1 |= USART_CR1_TXEIE;

        sent++;
    }
    return sent;
}


static ssize_t rs232_read_r(struct _reent *r, int fd, void *ptr, size_t len)
{
    // Blocking wait for the first char
    //
    unsigned timeout = portMAX_DELAY;

    char *dest = ptr;
    int  received = 0;
    static char last_c;

    while (len--) {
        char c;
        if (!xQueueReceive(rx_queue, &c, timeout))
            break;

        // Convert terminal CRLF to c-newline
        //
        if (c == '\n' && last_c == '\r')
            if (!xQueueReceive(rx_queue, &c, timeout))
                break;

        if (c == '\r')
            *dest++ = '\n';
        else
            *dest++ = c;

        last_c  = c;
        timeout = 0;
        received++;
    }

    return received;
}


static ssize_t rs232_chars_avail_r(struct _reent *r, int fd)
{
    return uxQueueMessagesWaiting(rx_queue);
}


static void rs232_init_uart(void)
{
    static int is_initialized;

    if (is_initialized)
        return;

    is_initialized = 1;

    // Enable peripheral clocks
    //
    RCC->AHBENR |= RCC_AHBPeriph_GPIOA;
    RCC->APB1ENR |= RCC_APB1Periph_USART2;

    USART_DeInit(USART2);

    // Initialize USART
    //
    USART_Init(USART2, &(USART_InitTypeDef) {
        .USART_BaudRate = RS232_BAUDRATE,
        .USART_WordLength = USART_WordLength_8b,
        .USART_StopBits = USART_StopBits_1,
        .USART_Parity = USART_Parity_No ,
        .USART_HardwareFlowControl = USART_HardwareFlowControl_None,
        .USART_Mode = USART_Mode_Rx | USART_Mode_Tx
    });

    // Initialize GPIO pins
    //
    // PA2  USART2_TX
    // PA3  USART2_RX
    //
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_7);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_7);

    GPIO_Init(GPIOA, &(GPIO_InitTypeDef) {
        .GPIO_Pin   = GPIO_Pin_2 | GPIO_Pin_3,
        .GPIO_Speed = GPIO_Speed_2MHz,
        .GPIO_Mode  = GPIO_Mode_AF,
        .GPIO_OType = GPIO_OType_PP,
        .GPIO_PuPd  = GPIO_PuPd_UP
    });

    USART_Cmd(USART2, ENABLE);
}


void rs232_poll_send(const char *ch)
{
    rs232_init_uart();

    while (*ch) {
        if (*ch == '\n') {
            while (!(USART2->ISR & USART_ISR_TXE));
            USART2->TDR = '\r';
        }
        while (!(USART2->ISR & USART_ISR_TXE));
        USART2->TDR = *ch++;
    }
}


/**
 * Initialize the rs232 UART.
 *
 */
void rs232_init(void)
{
    rs232_init_uart();

    if (!rx_queue) rx_queue = xQueueCreate(RX_QUEUE_SIZE, 1);
    if (!tx_queue) tx_queue = xQueueCreate(TX_QUEUE_SIZE, 1);

    NVIC_Init(&(NVIC_InitTypeDef) {
        .NVIC_IRQChannel = USART2_IRQn,
        .NVIC_IRQChannelPreemptionPriority =
                configLIBRARY_LOWEST_INTERRUPT_PRIORITY,
        .NVIC_IRQChannelSubPriority = 0,
        .NVIC_IRQChannelCmd = ENABLE
    });

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    dev_register("rs232", &term_rs232_ops);
}


struct file_ops term_rs232_ops = {
    .read_r        = rs232_read_r,
    .write_r       = rs232_write_r,
    .chars_avail_r = rs232_chars_avail_r
};


// -------------------- Shell commands --------------------
//
static void cmd_rs232_stats(void)
{
    printf("rx_bytes:   %10lu\n", rs232_stats.rx_bytes);
    printf("rx_overrun: %10lu\n", rs232_stats.rx_overrun);
    printf("tx_bytes:   %10lu\n", rs232_stats.tx_bytes);
    printf("tx_overrun: %10lu\n", rs232_stats.tx_overrun);
}

SHELL_CMD(rs232_stats, (cmdfunc_t) cmd_rs232_stats, "show rs232 statistics")
