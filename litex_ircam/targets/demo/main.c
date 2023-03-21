// This file is Copyright (c) 2020 Florent Kermarrec <florent@enjoy-digital.fr>
// License: BSD

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <irq.h>
#include <libbase/uart.h>
#include <libbase/console.h>
#include <generated/csr.h>

/*-----------------------------------------------------------------------*/
/* Uart                                                                  */
/*-----------------------------------------------------------------------*/

static char *readstr(void)
{
    char c[2];
    static char s[64];
    static int ptr = 0;

    if(readchar_nonblock()) {
        c[0] = getchar();
        c[1] = 0;
        switch(c[0]) {
            case 0x7f:
            case 0x08:
                if(ptr > 0) {
                    ptr--;
                    fputs("\x08 \x08", stdout);
                }
                break;
            case 0x07:
                break;
            case '\r':
            case '\n':
                s[ptr] = 0x00;
                fputs("\n", stdout);
                ptr = 0;
                return s;
            default:
                if(ptr >= (sizeof(s) - 1))
                    break;
                fputs(c, stdout);
                s[ptr] = c[0];
                ptr++;
                break;
        }
    }

    return NULL;
}

static char *get_token(char **str)
{
    char *c, *d;

    c = (char *)strchr(*str, ' ');
    if(c == NULL) {
        d = *str;
        *str = *str+strlen(*str);
        return d;
    }
    *c = 0;
    d = *str;
    *str = c+1;
    return d;
}

static void prompt(void)
{
    printf("\e[92;1mlitex-demo-app\e[0m> ");
}

/*-----------------------------------------------------------------------*/
/* Help                                                                  */
/*-----------------------------------------------------------------------*/

static void help(void)
{
    puts("\nLiteX minimal demo app built "__DATE__" "__TIME__"\n");
    puts("Available commands:");
    puts("help               - Show this command");
    puts("reboot             - Reboot CPU");
#ifdef CSR_LEDS_BASE
    puts("led                - Led demo");
#endif
    puts("donut              - Spinning Donut demo");
    puts("helloc             - Hello C");
#ifdef WITH_CXX
    puts("hellocpp           - Hello C++");
#endif
}

/*-----------------------------------------------------------------------*/
/* Commands                                                              */
/*-----------------------------------------------------------------------*/

static void reboot_cmd(void)
{
    ctrl_reset_write(1);
}

#ifdef CSR_LEDS_BASE
static void led_cmd(void)
{
    int i;
    printf("Led demo...\n");

    printf("Counter mode...\n");
    for(i=0; i<32; i++) {
        leds_out_write(i);
        busy_wait(100);
    }

    printf("Shift mode...\n");
    for(i=0; i<4; i++) {
        leds_out_write(1<<i);
        busy_wait(200);
    }
    for(i=0; i<4; i++) {
        leds_out_write(1<<(3-i));
        busy_wait(200);
    }

    printf("Dance mode...\n");
    for(i=0; i<4; i++) {
        leds_out_write(0x55);
        busy_wait(200);
        leds_out_write(0xaa);
        busy_wait(200);
    }
}
#endif

extern void donut(void);

static void donut_cmd(void)
{
    printf("Donut demo...\n");
    donut();
}

extern void helloc(void);

static void helloc_cmd(void)
{
    printf("Hello C demo...\n");
    helloc();
}

#ifdef WITH_CXX
extern void hellocpp(void);

static void hellocpp_cmd(void)
{
    printf("Hello C++ demo...\n");
    hellocpp();
}
#endif

#define SPI_CONTROL_START   (1 << 0)
#define SPI_CONTROL_LENGTH  (1 << 8)
#define SPI_STATUS_DONE     (1 << 0)
#define SPI_CS_MCP2515      (0)

static void spi_transfer(const uint8_t *tx, uint8_t *rx, int n)
{
    spi_cs_write(1 << SPI_CS_MCP2515);
    if (tx != NULL) {
        uint8_t buf[16] = {};
        
        // register is big-endian, MSB-first, left-aligned
        // So we need to swap byte order, but start at the first word.
        printf("MOSI: ");
        for (int i = 0; i < n; ++i) {
            printf("%02x ", tx[i]);
            buf[i^3] = tx[i];
        }
        printf("\n");
        
        // Need to write as 32-bit
        *(volatile unsigned int*)(CSR_SPI_MOSI_ADDR + 0) = *(unsigned int*)(buf + 0);
        *(volatile unsigned int*)(CSR_SPI_MOSI_ADDR + 4) = *(unsigned int*)(buf + 4);
        *(volatile unsigned int*)(CSR_SPI_MOSI_ADDR + 8) = *(unsigned int*)(buf + 8);
        *(volatile unsigned int*)(CSR_SPI_MOSI_ADDR + 12) = *(unsigned int*)(buf + 12);
    }

    spi_control_write(((n * 8) * SPI_CONTROL_LENGTH) | SPI_CONTROL_START);
    while (!(spi_control_read() & SPI_STATUS_DONE));

    if (rx != NULL) {
        // MISO is big-endian, MSB-first but right-aligned.
        uint8_t buf[16] = {};

        *(unsigned int*)(buf + 0) = *(volatile unsigned int*)(CSR_SPI_MISO_ADDR + 0);
        *(unsigned int*)(buf + 4) = *(volatile unsigned int*)(CSR_SPI_MISO_ADDR + 4);
        *(unsigned int*)(buf + 8) = *(volatile unsigned int*)(CSR_SPI_MISO_ADDR + 8);
        *(unsigned int*)(buf + 12) = *(volatile unsigned int*)(CSR_SPI_MISO_ADDR + 12);
        
        printf("MISO: ");
        for (int i = 0; i < n; ++i) {
            rx[i] = buf[(16-n + i) ^ 3];
            printf("%02x ", rx[i]);
        }
        printf("\n");
    }
}

#define RXBnCTRL_RXM_STD 			(0x20)
#define RXBnCTRL_RXM_EXT 			(0x40)
#define RXBnCTRL_RXM_STDEXT 		(0x00)
#define RXBnCTRL_RXM_MASK 			(0x60)
#define RXBnCTRL_RTR 			    (0x08)
#define RXB0CTRL_BUKT 			    (0x04)
#define RXB0CTRL_FILHIT_MASK 		(0x03)
#define RXB1CTRL_FILHIT_MASK 		(0x07)
#define RXB0CTRL_FILHIT 			(0x00)
#define RXB1CTRL_FILHIT 			(0x01)

#define MCP_SIDH 	(0)
#define MCP_SIDL 	(1)
#define MCP_EID8 	(2)
#define MCP_EID0 	(3)
#define MCP_DLC 	(4)
#define MCP_DATA 	(5)

#define STAT_RXIF_MASK (STAT.STAT_RX0IF | STAT.STAT_RX1IF)

#define MCP_RXF0SIDH			(0x00)
#define MCP_RXF0SIDL			(0x01)
#define MCP_RXF0EID8			(0x02)
#define MCP_RXF0EID0			(0x03)
#define MCP_RXF1SIDH			(0x04)
#define MCP_RXF1SIDL			(0x05)
#define MCP_RXF1EID8			(0x06)
#define MCP_RXF1EID0			(0x07)
#define MCP_RXF2SIDH			(0x08)
#define MCP_RXF2SIDL			(0x09)
#define MCP_RXF2EID8			(0x0A)
#define MCP_RXF2EID0			(0x0B)
#define MCP_CANSTAT				(0x0E)
#define MCP_CANCTRL				(0x0F)
#define MCP_RXF3SIDH			(0x10)
#define MCP_RXF3SIDL			(0x11)
#define MCP_RXF3EID8			(0x12)
#define MCP_RXF3EID0			(0x13)
#define MCP_RXF4SIDH			(0x14)
#define MCP_RXF4SIDL			(0x15)
#define MCP_RXF4EID8			(0x16)
#define MCP_RXF4EID0			(0x17)
#define MCP_RXF5SIDH			(0x18)
#define MCP_RXF5SIDL			(0x19)
#define MCP_RXF5EID8			(0x1A)
#define MCP_RXF5EID0			(0x1B)
#define MCP_TEC			    	(0x1C)
#define MCP_REC			    	(0x1D)
#define MCP_RXM0SIDH			(0x20)
#define MCP_RXM0SIDL			(0x21)
#define MCP_RXM0EID8			(0x22)
#define MCP_RXM0EID0			(0x23)
#define MCP_RXM1SIDH			(0x24)
#define MCP_RXM1SIDL			(0x25)
#define MCP_RXM1EID8			(0x26)
#define MCP_RXM1EID0			(0x27)
#define MCP_CNF3				(0x28)
#define MCP_CNF2				(0x29)
#define MCP_CNF1				(0x2A)
#define MCP_CANINTE				(0x2B)
#define MCP_CANINTF				(0x2C)
#define MCP_EFLG				(0x2D)
#define MCP_TXB0CTRL			(0x30)
#define MCP_TXB0SIDH			(0x31)
#define MCP_TXB0SIDL			(0x32)
#define MCP_TXB0EID8			(0x33)
#define MCP_TXB0EID0			(0x34)
#define MCP_TXB0DLC				(0x35)
#define MCP_TXB0DATA			(0x36)
#define MCP_TXB1CTRL			(0x40)
#define MCP_TXB1SIDH			(0x41)
#define MCP_TXB1SIDL			(0x42)
#define MCP_TXB1EID8			(0x43)
#define MCP_TXB1EID0			(0x44)
#define MCP_TXB1DLC				(0x45)
#define MCP_TXB1DATA			(0x46)
#define MCP_TXB2CTRL			(0x50)
#define MCP_TXB2SIDH			(0x51)
#define MCP_TXB2SIDL			(0x52)
#define MCP_TXB2EID8			(0x53)
#define MCP_TXB2EID0			(0x54)
#define MCP_TXB2DLC				(0x55)
#define MCP_TXB2DATA			(0x56)
#define MCP_RXB0CTRL			(0x60)
#define MCP_RXB0SIDH			(0x61)
#define MCP_RXB0SIDL			(0x62)
#define MCP_RXB0EID8			(0x63)
#define MCP_RXB0EID0			(0x64)
#define MCP_RXB0DLC				(0x65)
#define MCP_RXB0DATA			(0x66)
#define MCP_RXB1CTRL			(0x70)
#define MCP_RXB1SIDH			(0x71)
#define MCP_RXB1SIDL			(0x72)
#define MCP_RXB1EID8			(0x73)
#define MCP_RXB1EID0			(0x74)
#define MCP_RXB1DLC				(0x75)
#define MCP_RXB1DATA			(0x76)

#define CANINTF_RX0IF               (0x01)
#define CANINTF_RX1IF               (0x02)
#define CANINTF_TX0IF               (0x04)
#define CANINTF_TX1IF               (0x08)
#define CANINTF_TX2IF               (0x10)
#define CANINTF_ERRIF               (0x20)
#define CANINTF_WAKIF               (0x40)
#define CANINTF_MERRF               (0x80)

#define INSTRUCTION_WRITE			(0x02)
#define INSTRUCTION_READ			(0x03)
#define INSTRUCTION_BITMOD			(0x05)
#define INSTRUCTION_LOAD_TX0		(0x40)
#define INSTRUCTION_LOAD_TX1		(0x42)
#define INSTRUCTION_LOAD_TX2		(0x44)
#define INSTRUCTION_RTS_TX0			(0x81)
#define INSTRUCTION_RTS_TX1			(0x82)
#define INSTRUCTION_RTS_TX2			(0x84)
#define INSTRUCTION_RTS_ALL			(0x87)
#define INSTRUCTION_READ_RX0		(0x90)
#define INSTRUCTION_READ_RX1		(0x94)
#define INSTRUCTION_READ_STATUS		(0xA0)
#define INSTRUCTION_RX_STATUS		(0xB0)
#define INSTRUCTION_RESET			(0xC0)

#define CANCTRL_REQOP_NORMAL        (0x00)
#define CANCTRL_REQOP_SLEEP         (0x20)
#define CANCTRL_REQOP_LOOPBACK      (0x40)
#define CANCTRL_REQOP_LISTENONLY    (0x60)
#define CANCTRL_REQOP_CONFIG        (0x80)
#define CANCTRL_REQOP_POWERUP       (0xE0)
#define CANSTAT_OPMOD               (0xE0)
#define CANCTRL_REQOP               (0xE0)

#define STAT_RX0IF                  (1 << 0)
#define STAT_RX1IF                  (1 << 1)

#define TXB_ABTF                    (0x40)
#define TXB_MLOA                    (0x20)
#define TXB_TXERR                   (0x10)
#define TXB_TXREQ                   (0x08)
#define TXB_TXIE                    (0x04)
#define TXB_TXP                     (0x03)


static void mcp2515_reset(void)
{
    uint8_t buf[1] = {INSTRUCTION_RESET};
    spi_transfer(buf, 0, 1);
}

static void mcp2515_set_regs(int reg, uint8_t *regs, int n)
{
    uint8_t buf[16] = {INSTRUCTION_WRITE, reg};
    memcpy(buf + 2, regs, n);
    spi_transfer(buf, 0, n);
}

static void mcp2515_set(int reg, uint8_t val)
{
    uint8_t buf[3] = {INSTRUCTION_WRITE, reg, val};
    spi_transfer(buf, NULL, 3);
}

static uint8_t mcp2515_get(int reg)
{
    uint8_t buf[3] = {INSTRUCTION_READ, reg, 0};
    spi_transfer(buf, buf, 3);
    return buf[2];
}

static uint8_t mcp2515_get_status(void)
{
    uint8_t buf[3] = {INSTRUCTION_READ_STATUS, 0};
    spi_transfer(buf, buf, 2);
    return buf[1];
}

static void mcp2515_get_regs(int reg, uint8_t *msg, int num)
{
    uint8_t buf[16] = {INSTRUCTION_READ, reg};
    spi_transfer(buf, buf, num + 2);
    memcpy(msg, buf + 2, num);
}

static void mcp2515_rmw(int reg, uint8_t mask, uint8_t data)
{
    uint8_t buf[4] = {INSTRUCTION_BITMOD, reg, mask, data};
    spi_transfer(buf, NULL, 4);
}

static void mcp2515_id(uint8_t *reg, int id)
{
    reg[MCP_SIDH] = id >> 3;
    reg[MCP_SIDL] = (id & 7) << 5;
    reg[MCP_EID0] = 0;
    reg[MCP_EID8] = 0;
}

static void mcp2515_set_filter(int filter, int id)
{
    uint8_t filt[6] = {INSTRUCTION_WRITE, filter, 0, 0, 0, 0};
    mcp2515_id(filt + 2, id);
    spi_transfer(filt, NULL, sizeof(filt));
}

static void mcp2515_set_filter_mask(int filter, int canid)
{
    uint8_t reg[4];
    mcp2515_id(reg, canid);
    mcp2515_set_regs(filter ? MCP_RXM1SIDH : MCP_RXM0SIDH, reg, 4);
}

static void mcp2515_set_mode(int mode)
{
    mcp2515_rmw(MCP_CANCTRL, CANCTRL_REQOP, mode);
    while (1) {
        if ((mcp2515_get(MCP_CANSTAT) & CANSTAT_OPMOD)  == mode) {
            break;
        }
    }
}

static void can_sendframe(int canid, uint8_t *msg, int len)
{
    printf(">>> CAN ID %04x DLC %d ", canid, len);
    for (int i = 0; i < len; ++i) printf("%02x ", msg[i]);
    printf("\n");
    while (1) {
        for (int txb = 0; txb < 3; ++txb) {
            int base = MCP_TXB0CTRL - MCP_TXB0CTRL;
            switch (txb) {
            case 0: base = MCP_TXB0CTRL - MCP_TXB0CTRL; break;
            case 1: base = MCP_TXB1CTRL - MCP_TXB1CTRL; break;
            case 2: base = MCP_TXB2CTRL - MCP_TXB2CTRL; break;
            }

            int ctrl = mcp2515_get(base + MCP_TXB0CTRL);
            printf("txb %d ctrl %02x\n", txb, ctrl);
            if ((ctrl & TXB_TXREQ) == 0) {
                uint8_t data[16];
                mcp2515_id(data, canid);
                data[MCP_DLC] = len;
                memcpy(data + MCP_DATA, msg, len);
                mcp2515_set_regs(base + MCP_TXB0SIDH, data, MCP_DATA + len);
                mcp2515_rmw(base + MCP_TXB0CTRL, TXB_TXREQ, TXB_TXREQ);
                return;
            }
        }
    }
}

static int can_receive(int *canid, uint8_t *msg, int *len)
{
    int status = mcp2515_get_status();

    int base = 0;
    uint8_t mask = 0;

    if (status & STAT_RX0IF) {
        base = MCP_RXB0CTRL - MCP_RXB0CTRL;
        mask = CANINTF_RX0IF;
    } else if (status & STAT_RX1IF) {
        base = MCP_RXB1CTRL - MCP_RXB0CTRL;
        mask = CANINTF_RX1IF;
    } else {
        return 0;
    }

    printf("received something in %d\n", base);

    uint8_t buf[12];
    mcp2515_get_regs(base + MCP_RXB0SIDH, buf, MCP_DATA + 8);

    *len = buf[MCP_DLC] & 0xF;
    *canid = (buf[MCP_SIDH] << 3) | ((buf[MCP_SIDL]>>5) & 7);
    memcpy(msg, buf + MCP_DATA, *len);
    printf("got %d bytes of can from id %04x\n", *len, *canid);

    // Ack the reception
    mcp2515_rmw(base + MCP_CANINTF, mask, 0);

    return 1;
}

static void isotp_send(int id, const uint8_t *msg, int len)
{
    uint8_t buf[8] = {0};
    if (len < 6)
    {
        buf[0] = 0x8B;
        buf[1] = len;
        memcpy(buf + 2, msg, len);
        can_sendframe(id, buf, 8);
    } else {
        buf[0] = 0x8B;
        buf[1] = 0x10 | (len >> 8);
        buf[2] = len & 0xFF;
        memcpy(buf + 3, msg, 5);
        can_sendframe(id, buf, 8);
        int index = 1;
        for (int offset = 5; offset < len; offset += 6) {
            buf[0] = 0x8B;
            buf[1] = 0x20 | index++;
            int rem = len - offset;
            if (rem > 6) rem = 6;
            memcpy(buf + 2, msg + offset, rem);
            can_sendframe(id, buf, 8);
        }
    }
}

static void uds_cmd(void)
{
    printf("lol UDS\n");
    
    mcp2515_reset();
    printf("wait a bit\n");
    
    uint8_t zeros[14];
    mcp2515_set_regs(MCP_TXB0CTRL, zeros, sizeof(zeros));
    mcp2515_set_regs(MCP_TXB1CTRL, zeros, sizeof(zeros));
    mcp2515_set_regs(MCP_TXB2CTRL, zeros, sizeof(zeros));
    mcp2515_set(MCP_RXB0CTRL, 0);
    mcp2515_set(MCP_RXB1CTRL, 0);
    mcp2515_set(MCP_CANINTE, CANINTF_RX0IF | CANINTF_RX1IF | CANINTF_ERRIF | CANINTF_MERRF);
    mcp2515_rmw(MCP_RXB0CTRL, RXBnCTRL_RXM_MASK | RXB0CTRL_BUKT | RXB0CTRL_FILHIT_MASK, RXBnCTRL_RXM_STDEXT | RXB0CTRL_BUKT | RXB0CTRL_FILHIT);
    mcp2515_rmw(MCP_RXB1CTRL, RXBnCTRL_RXM_MASK | RXB1CTRL_FILHIT_MASK, RXBnCTRL_RXM_STDEXT | RXB1CTRL_FILHIT);
    mcp2515_set_filter(MCP_RXF0SIDH, 0);
    mcp2515_set_filter(MCP_RXF1SIDH, 0);
    mcp2515_set_filter(MCP_RXF2SIDH, 0);
    mcp2515_set_filter(MCP_RXF3SIDH, 0);
    mcp2515_set_filter(MCP_RXF4SIDH, 0);
    mcp2515_set_filter_mask(MCP_RXM0SIDH, 0);
    mcp2515_set_filter_mask(MCP_RXM1SIDH, 0);
    
    
    mcp2515_set(MCP_CNF1, 0x01);
    mcp2515_set(MCP_CNF2, 0xb4);
    mcp2515_set(MCP_CNF3, 0x86);
    
    mcp2515_set_mode(CANCTRL_REQOP_NORMAL);
    uint8_t  get_sn[] = {0x27, 0x01};
    isotp_send(0x657, get_sn, 2);

    for (int i = 0; i < 100; ++i)
    {
        uint8_t buf[8];
        int can_id;
        int dlc;
        if (can_receive(&can_id, buf, &dlc)) {
            printf("received %04x ", can_id);
            for (int i = 0; i < dlc; ++i)
            {
                printf("%02x", buf[i]);
            }
            printf("\n");
            break;
        }
    }

}

/*-----------------------------------------------------------------------*/
/* Console service / Main                                                */
/*-----------------------------------------------------------------------*/

static void console_service(void)
{
    char *str;
    char *token;

    str = readstr();
    if(str == NULL) return;
    token = get_token(&str);
    if(strcmp(token, "help") == 0)
        help();
    else if(strcmp(token, "reboot") == 0)
        reboot_cmd();
#ifdef CSR_LEDS_BASE
    else if(strcmp(token, "led") == 0)
        led_cmd();
#endif
    else if(strcmp(token, "helloc") == 0)
        helloc_cmd();
    else if(strcmp(token, "uds") == 0)
        uds_cmd();
#ifdef WITH_CXX
    else if(strcmp(token, "hellocpp") == 0)
        hellocpp_cmd();
#endif
    prompt();
}

int main(void)
{
#ifdef CONFIG_CPU_HAS_INTERRUPT
    irq_setmask(0);
    irq_setie(1);
#endif
    uart_init();

    help();
    prompt();

    while(1) {
        console_service();
    }

    return 0;
}
