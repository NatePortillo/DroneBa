#include "dw3000.h"
#include "BluetoothSerial.h"

 
#define APP_NAME "DS TWR INIT v1.0"
 
/* SPI COMMUNICATION PINS*/ 
const uint8_t PIN_RST = 27; //RESET PIN
const uint8_t PIN_IRQ = 34; //IRQ PIN
const uint8_t PIN_SS = 4; // SPI SELECT PIN
 
/*------------------UWB CONFIGURATION---------------------- */
static dwt_config_t config = {
  5,               /* Channel number. */
  DWT_PLEN_128,    /* Preamble length. Used in TX only. */
  DWT_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */
  9,               /* TX preamble code. Used in TX only. */
  9,               /* RX preamble code. Used in RX only. */
  1,               /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
  DWT_BR_6M8,      /* Data rate. */
  DWT_PHRMODE_STD, /* PHY header mode. */
  DWT_PHRRATE_STD, /* PHY header rate. */
  (129 + 8 - 8),   /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
  DWT_STS_MODE_OFF, /* STS disabled */
  DWT_STS_LEN_64,/* STS length see allowed values in Enum dwt_sts_lengths_e */
  DWT_PDOA_M0      /* PDOA mode off */
};
/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 500


/* Default antenna delay values for 64 MHz PRF. See NOTE 1 below. */
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

/* Frames used in the ranging process. See NOTE 2 below. */
static uint8_t tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21};
static uint8_t rx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0};
static uint8_t tx_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/* Length of the common part of the message (up to and including the function code, see NOTE 2 below). */
#define ALL_MSG_COMMON_LEN 10

/* Indexes to access some of the fields in the frames defined above. */
#define ALL_MSG_SN_IDX 2
#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_IDX 14
#define FINAL_MSG_FINAL_TX_TS_IDX 18

/* Frame sequence number, incremented after each transmission. */
static uint8_t frame_seq_nb = 0;

/* Buffer to store received response message.
 * Its size is adjusted to longest frame that this example code 
 * is supposed to handle. */
#define RX_BUF_LEN 20

static uint8_t rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference so that it can be 
 * examined at a debug breakpoint. */
static uint32_t status_reg = 0;

/* Delay between frames, in UWB microseconds. See NOTE 4 below. */
/* This is the delay from the end of the frame transmission to the enable of 
 * the receiver, as programmed for the DW IC's wait for response feature. */
#define POLL_TX_TO_RESP_RX_DLY_UUS 700
/* This is the delay from Frame RX timestamp to TX reply timestamp used for 
 * calculating/setting the DW IC's delayed TX function. This includes the
 * frame length of approximately 190 us with above configuration. */
#define RESP_RX_TO_FINAL_TX_DLY_UUS 700
/* Receive response timeout. See NOTE 5 below. */
#define RESP_RX_TIMEOUT_UUS 300
/* Preamble timeout, in multiple of PAC size. See NOTE 7 below. */
#define PRE_TIMEOUT 5

/* Time-stamps of frames transmission/reception, expressed in device time units. */
static uint64_t poll_tx_ts;
static uint64_t resp_rx_ts;
static uint64_t final_tx_ts;

/* Values for the PG_DELAY and TX_POWER registers reflect the bandwidth and power 
 * of the spectrum at the current temperature. 
 * These values can be calibrated prior to taking reference measurements.
 * See NOTE 8 below. */
extern dwt_txconfig_t txconfig_options;
/*--------------------------BLUETOOTH CONFIGURATION---------------------------*/
BluetoothSerial SerialBT;

void setup()
{
  /*--------------------------UART SEND CONFIGURATION-----------------------*/
  UART_init();
  /*--------------------------UWB CONFIGURATION-----------------------------*/
  test_run_info((unsigned char *)APP_NAME);
  /*--------------------------SPI CONFIGURATION-----------------------------*/
  spiBegin(PIN_IRQ, PIN_RST);
  spiSelect(PIN_SS);
  /*--------------------------BLUETOOTH CONFIGURATION---------------------------*/
   SerialBT.begin("ESP32_Transmitter"); //Bluetooth device name 
   Serial.println("The device started, now you can pair it with bluetooth!");
   Sleep(2);
  /*-------------------UWB CONFIGURATION CONTINUED---------------------------------*/
  while (!dwt_checkidlerc()) {  // Need to make sure DW IC is in IDLE_RC before proceeding
    UART_puts("IDLE FAILED\r\n");
    while (1) ;
  }

  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
      UART_puts("INIT FAILED\r\n");      
      while (1) { /* spin */ };
  }
  if (dwt_configure(&config)) {
      UART_puts("CONFIG FAILED\r\n");
      while (1) { /* spin */ };
  }
  dwt_configuretxrf(&txconfig_options); //Configure the TX spectrum parameters
  dwt_setrxantennadelay(RX_ANT_DLY); // Apply default antenna delay value
  dwt_settxantennadelay(TX_ANT_DLY);
  dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS); //Set expected response's delay and timeout
  dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
  dwt_setpreambledetecttimeout(PRE_TIMEOUT);
  dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE); //DEBUG LEDs
  dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);
}
    void loop()  {
      while (1) {
        tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb; //Write frame data to DW IC and prepare transmission
        dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0); /* Zero offset in TX buffer. */
        dwt_writetxfctrl(sizeof(tx_poll_msg)+FCS_LEN, 0, 1); /* Zero offset in TX buffer, ranging. */
        /* Start transmission, indicating that a response is expected so that
         * reception is enabled automatically after the frame is sent and the 
         * delay set by dwt_setrxaftertxdelay() has elapsed. */
        dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
        /* We assume that the transmission is achieved correctly, poll for 
         * reception of a frame or error/timeout. */
        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & 
                                               (SYS_STATUS_RXFCG_BIT_MASK | 
                                                SYS_STATUS_ALL_RX_TO      | 
                                                SYS_STATUS_ALL_RX_ERR)))
        { /* spin */ };
        frame_seq_nb++;
        if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {

            /* Clear good RX frame event and TX frame sent in the DW IC status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_TXFRS_BIT_MASK);
            /* A frame has been received, read it into the local buffer. */
            uint32_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & FRAME_LEN_MAX_EX;
            if (frame_len <= RX_BUF_LEN) {
                dwt_readrxdata(rx_buffer, frame_len, 0);
            }
            /* Check that the frame is the expected response from the 
             * companion "DS TWR responder" example.
             * As the sequence number field of the frame is not relevant, 
             * it is cleared to simplify the validation of the frame. */
            rx_buffer[ALL_MSG_SN_IDX] = 0;
            if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0) {
                uint32_t final_tx_time;
                /* Retrieve poll transmission and response reception timestamp. */
                poll_tx_ts = get_tx_timestamp_u64();
                resp_rx_ts = get_rx_timestamp_u64();
                /* Compute final message transmission time. See NOTE 11 below. */
                final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
                dwt_setdelayedtrxtime(final_tx_time);
                /* Final TX timestamp is the transmission time we programmed 
                 * plus the TX antenna delay. */
                final_tx_ts = (((uint64_t)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;
                /* Write all timestamps in the final message. See NOTE 12 below. */
                final_msg_set_ts(&tx_final_msg[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
                final_msg_set_ts(&tx_final_msg[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts);
                final_msg_set_ts(&tx_final_msg[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);
                /* Write and send final message. See NOTE 9 below. */
                tx_final_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
                dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0); /* Zero offset in TX buffer. */
                dwt_writetxfctrl(sizeof(tx_final_msg)+FCS_LEN, 0, 1); /* Zero offset in TX buffer, ranging bit set. */
                /* If dwt_starttx() returns an error, abandon this ranging exchange and
                 * proceed to the next one. See NOTE 13 below. */
                int ret = dwt_starttx(DWT_START_TX_DELAYED);
                if (ret == DWT_SUCCESS) {
                    /* Poll DW IC until TX frame sent event set. See NOTE 10 below. */
                    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK))
                    { /* spin */ };

                    /* Clear TXFRS event. */
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);

                    /* Increment frame sequence number after transmission of the 
                     * final message (modulo 256). */
                    frame_seq_nb++;
                }
            }
        }
        else {
            /* Clear RX error/timeout events in the DW IC status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_TXFRS_BIT_MASK);
        }

        /* Execute a delay between ranging exchanges. */
        Sleep(RNG_DELAY_MS);

      /*--------------------------BLUETOOTH CONFIGURATION---------------------------*/
      if (SerialBT.available()) {
        Serial.println("I am available"); //DEBUG - BT Available and successfully connected.
        String command = SerialBT.readStringUntil('\n'); //Read BT data 
        Serial.println(String(command));
        command.trim(); // This removes any whitespace or newline characters from both ends of the string

        if (command == "0xCC") {
          Serial.println("Distance is below 100"); //DEBUG
          digitalWrite(2, HIGH);

        }else{
          Serial.println("Distance is above 100"); //DEBUG
          digitalWrite(2, LOW);
        }
      }
      if (Serial.available()) { // Sending status back to Master
        SerialBT.write(Serial.read());
      }
    }
}
