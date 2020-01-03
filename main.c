/*
 * SPI write example.
 */

#include "gos.h"

/*
 * Test switches:
 *
 * TESTPROG 	Loads a test program from the mgm13_bluetooth_bin array to the
 * 				bluetooth target processor and runs it.
 *
 * TESTDATA     Tests sending of random data packets to the bluetooth target
 * 				processor. If the target is configured to echo all packets, this
 * 				also an upstream test.
 *
 * Note that without either of the above flags set, the program will simply sign
 * and soft halt.
 *
 */
//#define TESTPROG        /* load test program */
#define TESTDATA		/* transmit test data */
//#define FILLINC			/* fill outgoing packets with incrementing data */
#define FILLRND			/* fill outgoing packets with random data */

#define CMDLEN 5        /* length of standard command */
#define PKTLEN (2*1024) /* length of max packet + overhead */

#define TPLEN (10*1024) /* test program length */


#define FLSSIZ 1024     /* basic flash packet size */

/* SPI protocol commands */
typedef enum {

    spicmd_noop,       /* 0: zero code, unused */
    spicmd_mts_len,    /* 1: master to slave packet length */
    spicmd_mts_pkt,    /* 2: master to slave packet transfer */
    spicmd_stm_len,    /* 3: slave to master packet length */
    spicmd_stm_pkt,    /* 4: slave to master packet transfer */
    spicmd_mts_gbs,    /* 5: go to bootstrap code */
    spicmd_mts_sps,    /* 6: master to slave, start reprogram sequence */
	spicmd_mts_ppd,    /* 7: master to slave, pass program data */
	spicmd_mts_pex,    /* 8: master to slave, execute program */
	spicmd_rsf = 0xfd, /* reprogram sequence fails */
	spicmd_nrp = 0xfe, /* slave to master status, no resident program */
    spicmd_err = 0xff  /* protocol faulted, reset */

} spicmd;

extern unsigned char mgm13_bluetooth_bin[];
extern unsigned int mgm13_bluetooth_bin_flashlen;

uint8_t cmdtxbuf[CMDLEN]; /* SPI transmit command buffer */
uint8_t cmdrxbuf[CMDLEN]; /* SPI receive command buffer */
uint8_t pkttxbuf[PKTLEN]; /* SPI transmit packet buffer */
uint8_t pktrxbuf[PKTLEN]; /* SPI receive packet buffer */

gos_spi_device_t spidev; /* handle to SPI device */

/* test program image */
//uint8_t testprog[TPLEN]; /* area for test program image */

/* test buffers */
uint8_t testtxbuf[PKTLEN]; /* outbound */
uint8_t testrxbuf[PKTLEN]; /* inbound */

/* crc random number generator buffer */
unsigned short randnum;

/* bluetooth IRQ service flag */
boolean slaveirq;

/*******************************************************************************

Calculate 16 bit CRC

Finds a 16 bit CRC according to CRC-CCIT-16 using a 0xFFFF starting word.
The block of bytes to be checked is given, with the length of the block.

*******************************************************************************/

unsigned short crc16(const unsigned char* data_p, unsigned length)

{

    unsigned char x;
    unsigned short crc = 0xFFFF;

    while (length--){

        x = crc >> 8 ^ *data_p++;
        x ^= x>>4;
        crc = (crc << 8) ^ ((unsigned short)(x << 12)) ^
              ((unsigned short)(x <<5)) ^ ((unsigned short)x);

    }

    return crc;

}

/*******************************************************************************

Calculate 32 bit CRC

Finds a 32 bit CRC using the same algorithm as Ethernet CRC. The block of bytes
to be checked is given, with the length of the block.

*******************************************************************************/

unsigned long crc32(const unsigned char* data, unsigned long length)

{

    unsigned int crc = 0xffffffff;	/* Initial value. */
    unsigned char current_octet;
    int bit;

    while(length-- > 0) {

	    current_octet = *data++;

	    for (bit = 8; --bit >= 0; current_octet >>= 1) {

	        if ((crc ^ current_octet) & 1) {

	            crc >>= 1;
	            crc ^= 0xedb88320U; /* ethernet polynomial */

	        } else crc >>= 1;

	    }

    }

    return (crc);

}

/*******************************************************************************

Bluetooth IRQ callback

Called when the Bluetooth slave signals an interrupt.

*******************************************************************************/

void bluetoothirq(void *dummy)

{

    slaveirq = true; /* set slave signaled upstream packet ready */

}

/*******************************************************************************

Run SPI transfer

Runs a transfer on the SPI bus. A SPI transfer is a simultaneous receive and
transmit operation. The dual DMAs are programmed, one for each of the slave
receive and transmit buffers, and with a matching length.

Note that the given buffers must contain 2 more bytes than the length specified,
due to the addition of a 16 bit CRC to the end.

The chkrcv parameter specifies if the recieve packet is to be CRC checked. The
protocol specifies that the length commands, 1 and 3, should not be checked,
since that impacts the arbitration between master to slave and slave to master
packets. Simply put, the receive buffer could be in any state of modification.

Returns nonzero if there was an error in the packet send or receive.

*******************************************************************************/

boolean spixfr(uint8_t* txbuf, uint8_t* rxbuf, int len, boolean chkrcv)

{

    unsigned short crc, rcrc;
    gos_spi_message_t sm;
    gos_result_t r;
    boolean fail;

    fail = false;
    crc = crc16(txbuf, len); /* calculate CRC on payload */
    txbuf[len] = crc >> 8; /* place CRC */
    txbuf[len+1] = crc & 0xff;

    /* set buffers and length */
    sm.tx_buffer = txbuf;
    sm.rx_buffer = rxbuf;
    sm.length = len+2;

    /* transmit */
    gos_gpio_set(GOS_GPIO_25, 0);
    r = gos_spi_transfer(&spidev, &sm, 1);
    /* we let the gos_spi_transfer() routine assert the chip select, but it has
       bug that releases it too early. So we handle the deassert */
    gos_gpio_set(GOS_GPIO_25, 1);
    if (r != 0) GOS_LOG("SPI transfer returns: %d", r);
    gos_spi_master_deassert(&spidev);

    if (chkrcv) { /* check received packet */

        /* check the received packet */
        crc = crc16(rxbuf, len); /* find payload crc */
        rcrc = rxbuf[len] << 8 | rxbuf[len+1]; /* pick up received payload */
        if (rxbuf[0] == spicmd_err) {

            GOS_LOG("Slave indicates error");
            fail = true;

        } else if (crc != rcrc) {

            GOS_LOG("Received CRC does not match calculated: %04x s/b %04x",
                    rcrc, crc);
            fail = true;

        }

    }
    /* delay to let slave do work (this is a measured critical delay!) */
    gos_rtos_delay_milliseconds(2);

    return (fail);

}

/*******************************************************************************

Initialize SPI channel

Initalizes the SPI channel to run.

*******************************************************************************/

void spiini(void)

{

    /* set GPIO 25, the chip select, for normal output */
	gos_gpio_init(GOS_GPIO_25,GOS_GPIO_OUTPUT_PUSH_PULL,1);

	/* init spi device */
    spidev.port = 1;
    spidev.bits = 8;
    /* GPIO 19 is JTAG TDI, we don't use that because we use the SWO debug
       interface. We have to assign the SPI driver a useless pin because Silabs
       bugs prevent both using their CS software as well as the code to tell it
       not to generate a CS - that causes it to fall over as well */
    spidev.chip_select = 19;
    spidev.speed = 1000000;
    spidev.lcd_bit8 = false;
    spidev.flags = GOS_SPI_MSB_FIRST | GOS_SPI_CLOCK_RISING_EDGE;

    slaveirq = false; /* set no irq active */
    /* set GPIO 20, the bluetooth IRQ, for normal output */
	gos_gpio_init(GOS_GPIO_20, GOS_GPIO_INPUT_PULL_UP, 1);
    /* enable interrupt from slave */
    gos_gpio_irq_enable(GOS_GPIO_20, GOS_GPIO_TRIGGER_FALLING_EDGE,
                        bluetoothirq, &slaveirq);

}

/*******************************************************************************

Transmit bluetooth packet

Called when a packet is to be transferred from master to slave. Sets up a
downstream #1 or MTS length command in the buffer, sets the state code to
#1 and starts the transfer. This is a blocking call.

The sequence to transfer is:

1. A #1 command is sent to request the length of the packet from the master.

2. A #2 command is sent to transfer the packet from master to slave.

*******************************************************************************/

void bluetooth_packet_transmit(uint8_t* buff, int len)

{

    /* send the MTS length command */
    cmdtxbuf[0] = spicmd_mts_len; /* place MTS length command */
    cmdtxbuf[1] = (len+3) >> 8; /* place length */
    cmdtxbuf[2] = (len+3) & 0xff;

    /* send length command */
    spixfr(cmdtxbuf, cmdrxbuf, 3, false);

    memcpy(pkttxbuf+1, buff, len); /* copy packet to transmit buffer */
    pkttxbuf[0] = spicmd_mts_pkt; /* place packet to slave command */

    /* send packet */
    spixfr(pkttxbuf, pktrxbuf, len+1, true);

}

/*******************************************************************************

Receive bluetooth packet

Called when a packet is to be transferred from slave to master. Sets up a
downstream #3 or STM length command in the buffer, and sets the state code to
#2 and starts the transfer. This is a blocking call. The length of the packet
received is returned.

Note that this call is usually made after an IRQ interrupt from the slave
occurs, signaling that the slave has a packet to transfer to the master.

The sequence to transfer is:

1. A #3 command is sent to request the length of the packet from the slave.

2. A #4 command is sent to transfer the packet from the slave.

The resulting packet is then copied to the address given .

*******************************************************************************/

void bluetooth_packet_receive(uint8_t* buff, int* len)

{

    int i;
    boolean fail;

    /* format the send length command. */
    cmdtxbuf[0] = spicmd_stm_len; /* place stm length command */
    cmdtxbuf[1] = 0; /* place dummy length */
    cmdtxbuf[2] = 0;

    *len = 0; /* set default length */

    /* send length command */
    fail = spixfr(cmdtxbuf, cmdrxbuf, 3, true);

    /* don't transfer on failed packet, the length could be wrong */
    if (!fail) {

        /* get actual length of inbound packet */
        *len = (cmdrxbuf[1] <<8 | cmdrxbuf[2])-3;
        /* check packet length invalid and limit if so to prevent crashing */
        if (*len < 0 || *len > PKTLEN-3) {

            GOS_LOG("Packet length specified is invalid, truncating");
            *len = PKTLEN-3;

        }

        /* format the outbound packet buffer */
        pkttxbuf[0] = spicmd_stm_pkt;
        /* clear payload for good measure */
        for (i = 0; i < *len; i++) pkttxbuf[i+1] = 0;

        /* receive packet */
        spixfr(pkttxbuf, pktrxbuf, 1+*len, true);

        /* check command was expected */
        if (pktrxbuf[0] != spicmd_stm_pkt) GOS_LOG("Command code mismatch");

        /* copy data back to caller */
        memcpy(buff, &pktrxbuf[1], *len);

    }

}

/*******************************************************************************

Transfer bluetooth program image

Given a program image base and length, the image is transferred to the bluetooth
processor to be burned into flash. This is followed by an instruction to execute
the image, and thus it will start without the need to reset the chip.

Note that the program image can and will overwrite the previous image in the
bluetooth controller. The bootstrap is resident in high memory and is not
normally reprogrammed.

The bluetooth chip is running either the bootstrap or the main program (the
state of this can be determined). If the main program is running, it will branch
to the bootstrap on receiving the reprogram command. If the bootstrap is
running, it will simply start the load and flash sequence. In any case, the
existing resident program is erased without issue.

The input program image must be an even multiple of the flash size, or 2048,
otherwise the programming operation can and will malfunction.

*******************************************************************************/

void load_resident(uint8_t* buff, int len)

{

    int i;
    unsigned long crc;

    printf("Downloading resident program: length: %d", len);

    gos_rtos_delay_milliseconds(1000);

    /* send the MTS go bootstrap command
       Note the target may already be in bootstrap, in which case this
       command is a no-op. */
    cmdtxbuf[0] = spicmd_mts_gbs; /* place MTS go bootstrap command */
    cmdtxbuf[1] = 0; /* place (dummy) length */
    cmdtxbuf[2] = 0;
    /* send command */
    spixfr(cmdtxbuf, cmdrxbuf, 3, false);

    /* give far program time to enter bootstrap */
    gos_rtos_delay_milliseconds(1000);

    /* send the MTS start reprogram sequence command */
    cmdtxbuf[0] = spicmd_mts_sps; /* place MTS start program sequence command */
    cmdtxbuf[1] = 0; /* place (dummy) length */
    cmdtxbuf[2] = 0;
    /* send command */
    spixfr(cmdtxbuf, cmdrxbuf, 3, false);

    /* burn flash sectors */
    for (i = 0; i < len; i += FLSSIZ) {

        memcpy(pkttxbuf+1, buff+i, FLSSIZ); /* copy packet to transmit buffer */
        pkttxbuf[0] = spicmd_mts_ppd; /* place packet to slave command */

        /* wait for IRQ, which signifies slave ready */
        while (!slaveirq)
            /* call back to OS to pick up IRQ events */
            gos_rtos_get_time();
        slaveirq = false;

        /* send packet */
        spixfr(pkttxbuf, pktrxbuf, FLSSIZ+1, true);

    }
    /* wait for IRQ, which signifies slave ready */
    while (!slaveirq)
        /* call back to OS to pick up IRQ events */
        gos_rtos_get_time();
    slaveirq = false;

    /* find the total program image CRC */
    crc = crc32(buff, len);
    printf("Sending execute resident program");

     /* send the MTS length command */
    cmdtxbuf[0] = spicmd_mts_pex; /* place MTS start program sequence command */
    cmdtxbuf[1] = crc >> 24 & 0xff; /* place CRC */
    cmdtxbuf[2] = crc >> 16 & 0xff;
    cmdtxbuf[3] = crc >> 8 & 0xff;
    cmdtxbuf[4] = crc & 0xff;
    /* send command */
    spixfr(cmdtxbuf, cmdrxbuf, FLSSIZ+1, false);

}

void gos_app_init(void)

{

    GOS_LOG("Start application: init spi");

    /* initialize SPI channel */
    spiini();

#ifdef TESTPROG

	{

		int i;

        i = 1;
	    while (1) {

		    load_resident(mgm13_bluetooth_bin, mgm13_bluetooth_bin_flashlen); /* load test program */
		    printf("Iteration: %d", i++);

	    }

	}

#endif

#ifdef TESTDATA

    {

        int i;
        int len;

#ifdef FILLINC
        /* pattern the test buffer with incrementing data */
        for (i = 0; i < TPLEN; i++) testprog[i] = i;
#endif

#ifdef FILLRND
        /* fill the outbound buffer with test data. We use the CRC generator as a
           random sequence generator */
        for (i = 0; i < 2*1024; i = i+2) {

            /* randomize */
            randnum = crc16((const unsigned char*) &randnum, sizeof(unsigned short));
            testtxbuf[i] = randnum >> 8;
            testtxbuf[i+1] = randnum & 0xff;

        }
#endif

        i = 0;
        while (1) {

printf("Transmit packet");
            bluetooth_packet_transmit(testtxbuf, i++/*2*1024*/); /* send packet */
            if (i == PKTLEN-3) i = 0;
            /* note that if the bluetooth processor is set to echo all packets,
               will come back here */
            if (slaveirq) { /* slave flags packet ready, go receive */

printf("Receive packet");
                slaveirq = false; /* clear irq */
                bluetooth_packet_receive(testrxbuf, &len); /* receive packet */

            }

        }

    }
#endif

    while (1); /* stop */

}
