#include "Adafruit_VL53L0X.h"
#include "dw3000.h"
#include "BluetoothSerial.h"
#include <HardwareSerial.h>

#define APP_NAME "DS TWR RESP v1.0"
 
//SPI COMMUNICATION
const uint8_t PIN_RST = 27; // reset pin
const uint8_t PIN_IRQ = 34; // irq pin
const uint8_t PIN_SS = 4; // spi select pin
 
/* Create structure for the configuration */
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
static uint8_t rx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21, 0, 0};
static uint8_t tx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0, 0, 0};
static uint8_t rx_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/* Length of the common part of the message */
#define ALL_MSG_COMMON_LEN 10

/* Index to access some of the fields in the frames involved in the process. */
#define ALL_MSG_SN_IDX 2
#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_IDX 14
#define FINAL_MSG_FINAL_TX_TS_IDX 18

/* Frame sequence number, incremented after each transmission. */
static uint8_t frame_seq_nb = 0;

/* Buffer to store received messages.
 * Its size is adjusted to longest frame that this example code is
 * supposed to handle. */
#define RX_BUF_LEN 24
static uint8_t rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference so that it can be
 * examined at a debug breakpoint. */
static uint32_t status_reg = 0;

/* Delay between frames, in UWB microseconds. See NOTE 4 below. */
/* This is the delay from Frame RX timestamp to TX reply timestamp used for
 * calculating/setting the DW IC's delayed TX function. This includes the
 * frame length of approximately 190 us with above configuration. */
#define POLL_RX_TO_RESP_TX_DLY_UUS 900

/* This is the delay from the end of the frame transmission to the enable of
 * the receiver, as programmed for the DW IC's wait for response feature. */
#define RESP_TX_TO_FINAL_RX_DLY_UUS 500

/* Receive final timeout. See NOTE 5 below. */
#define FINAL_RX_TIMEOUT_UUS 220

/* Preamble timeout, in multiple of PAC size. See NOTE 6 below. */
#define PRE_TIMEOUT 5

/* Timestamps of frames transmission/reception. */
static uint64_t poll_rx_ts;
static uint64_t resp_tx_ts;
static uint64_t final_rx_ts;

/* Hold copies of computed time of flight and distance here for reference
 * so that it can be examined at a debug breakpoint. */
static double tof;
static double distance;

/* Values for the PG_DELAY and TX_POWER registers reflect the bandwidth and
 * power of the spectrum at the current temperature. These values can be
 * calibrated prior to taking reference measurements.
 * See NOTE 2 below. */
extern dwt_txconfig_t txconfig_options;

/*--------------------------BLUETOOTH CONFIGURATION---------------------------*/
BluetoothSerial SerialBT;
int i = 0;

String MACadd = "AA:BB:CC:11:22:33";
uint8_t address[6]  = {0xAA, 0xBB, 0xCC, 0x11, 0x22, 0x33};

String name = "ESP32_Transmitter"; //Other name of the bluetooth
char *pin = "1234"; //<- standard pin would be provided by default
bool connected;
/*------------------------------------------------------------------------------*/

/*--------------------------VL53L TOF SENSOR CONFIGURATION-----------------------*/
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
float vlxMeasurement = 0;
uint8_t vlxMeasurement_Status = 0xAA;
/*------------------------------------------------------------------------------*/

/*--------------------------UART SEND CONFIGURATION-----------------------*/
#define START_BYTE 0xAA
#define END_BYTE 0xBB

uint8_t closeRange = 0xEE;
uint8_t pynqDataPacket[13];
HardwareSerial SerialPort(2); // Using UART1
/*------------------------------------------------------------------------------*/

void setup() 
{
/*--------------------------UART SEND CONFIGURATION-----------------------*/
  SerialPort.begin(115200, SERIAL_8N1, 16, 17); 
/*------------------------------------------------------------------------------*/

/*--------------------------SPI CONFIGURATION-----------------------------*/
  UART_init();
  test_run_info((unsigned char *)APP_NAME);
  spiBegin(PIN_IRQ, PIN_RST);
  spiSelect(PIN_SS);

/*--------------------------BLUETOOTH CONNECT---------------------------*/
  SerialBT.begin("ESP32testm", true); 
  Serial.println("The device started in master mode, make sure remote BT device is on!");
  connected = SerialBT.connect(name);
    if(connected) {
    Serial.println("Connected Succesfully!");
  } else {
    while(!SerialBT.connected(10000)) {
      Serial.println("Failed to connect. Make sure remote device is available and in range, then restart app."); 
    }
  }
  if (SerialBT.disconnect()) {
    Serial.println("Disconnected Succesfully!");
  }
  // this would reconnect to the name(will use address, if resolved) or address used with connect(name/address).
  SerialBT.connect();
  /*-------------------------------------------------------------------*/
    Sleep(2);
    /* Need to make sure DW IC is in IDLE_RC before proceeding */
    while (!dwt_checkidlerc()) {
          UART_puts("IDLE FAILED\r\n");
    while (1) ;

     };
    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
       // LOG_ERR("INIT FAILED");
        while (1) { /* spin */ };
    }
    /* Configure DW IC. See NOTE 15 below. */
    if (dwt_configure(&config)) {
       // LOG_ERR("CONFIG FAILED");
        while (1) { /* spin */ };
    }

/*---------------------------CONFIGURE PARAMETERS---------------------------------*/
    /* Configure the TX spectrum parameters (power, PG delay and PG count) */
    dwt_configuretxrf(&txconfig_options);
    /* Apply default antenna delay value. See NOTE 1 below. */
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);
    /* Next can enable TX/RX states output on GPIOs 5 and 6 to help debug,
     * and also TX/RX LEDs.
     * Note, in real low power applications the LEDs should not be used.
     */
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);
/*--------------------------VL53L INITIALIZE-----------------------------*/
  Serial.begin(115200);

  // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(1);
  }
  
  Serial.println("Adafruit VL53L0X test");
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }
}    

    /* Loop forever responding to ranging requests. */
    void loop (){
      while (1){
        dwt_setpreambledetecttimeout(0);
        /* Clear reception timeout to start next ranging process. */
        dwt_setrxtimeout(0);

        /* Activate reception immediately. */
        dwt_rxenable(DWT_START_RX_IMMEDIATE);

        /* Poll for reception of a frame or error/timeout. See NOTE 8 below. */
        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK |SYS_STATUS_ALL_RX_TO |SYS_STATUS_ALL_RX_ERR)))
        { /* spin */ };

        if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {

            /* Clear good RX frame event in the DW IC status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

            /* A frame has been received, read it into the local buffer. */
            uint32_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & FRAME_LEN_MAX_EX;
            if (frame_len <= RX_BUF_LEN) {
                dwt_readrxdata(rx_buffer, frame_len, 0);
            }

            /* Check that the frame is a poll sent by "DS TWR initiator" example.
             * As the sequence number field of the frame is not relevant, it
             * is cleared to simplify the validation of the frame.
             */
            rx_buffer[ALL_MSG_SN_IDX] = 0;
            if (memcmp(rx_buffer, rx_poll_msg, ALL_MSG_COMMON_LEN) == 0) {

                uint32_t resp_tx_time;

                /* Retrieve poll reception timestamp. */
                poll_rx_ts = get_rx_timestamp_u64();

                /* Set send time for response. See NOTE 9 below. */
                resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
                dwt_setdelayedtrxtime(resp_tx_time);

                /* Set expected delay and timeout for final message reception. See NOTE 4 and 5 below. */
                dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
                dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);

                /* Set preamble timeout for expected frames. See NOTE 6 below. */
                dwt_setpreambledetecttimeout(PRE_TIMEOUT);

                /* Write and send the response message. See NOTE 10 below.*/
                tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
                dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0); /* Zero offset in TX buffer. */
                dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1); /* Zero offset in TX buffer, ranging. */
                int ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);

                /* If dwt_starttx() returns an error, abandon this ranging
                 * exchange and proceed to the next one. See NOTE 11 below. */
                if (ret == DWT_ERROR) {
                    continue;
                }

                /* Poll for reception of expected "final" frame or error/timeout.
                 * See NOTE 8 below.
                 */
                while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
                                                       (SYS_STATUS_RXFCG_BIT_MASK |
                                                        SYS_STATUS_ALL_RX_TO |
                                                        SYS_STATUS_ALL_RX_ERR)))
                { /* spin */ };

                /* Increment frame sequence number after transmission of the
                 * response message (modulo 256).
                 */
                frame_seq_nb++;

                if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {

                    /* Clear good RX frame event and TX frame sent in the DW IC status register. */
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_TXFRS_BIT_MASK);

                    /* A frame has been received, read it into the local buffer. */
                    frame_len = dwt_read32bitreg(RX_FINFO_ID) & FRAME_LEN_MAX_EX;
                    if (frame_len <= RX_BUF_LEN) {
                        dwt_readrxdata(rx_buffer, frame_len, 0);
                    }

                    /* Check that the frame is a final message sent by
                     * "DS TWR initiator" example.
                     * As the sequence number field of the frame is not used in
                     * this example, it can be zeroed to ease the validation of
                     * the frame.
                     */
                    rx_buffer[ALL_MSG_SN_IDX] = 0;
                    if (memcmp(rx_buffer, rx_final_msg, ALL_MSG_COMMON_LEN) == 0) {

                        uint32_t poll_tx_ts, resp_rx_ts, final_tx_ts;
                        uint32_t poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
                        double Ra, Rb, Da, Db;
                        int64_t tof_dtu;

                        /* Retrieve response transmission and final
                         * reception timestamps. */
                        resp_tx_ts = get_tx_timestamp_u64();
                        final_rx_ts = get_rx_timestamp_u64();

                        /* Get timestamps embedded in the final message. */
                        final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts);
                        final_msg_get_ts(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX], &resp_rx_ts);
                        final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts);

                        /* Compute time of flight. 32-bit subtractions give
                         * correct answers even if clock has wrapped.
                         * See NOTE 12 below. */
                        poll_rx_ts_32 = (uint32_t)poll_rx_ts;
                        resp_tx_ts_32 = (uint32_t)resp_tx_ts;
                        final_rx_ts_32 = (uint32_t)final_rx_ts;
                        Ra = (double)(resp_rx_ts - poll_tx_ts);
                        Rb = (double)(final_rx_ts_32 - resp_tx_ts_32);
                        Da = (double)(final_tx_ts - resp_rx_ts);
                        Db = (double)(resp_tx_ts_32 - poll_rx_ts_32);
                        tof_dtu = (int64_t)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));

                        tof = tof_dtu * DWT_TIME_UNITS;
                        distance = tof * SPEED_OF_LIGHT;
                        Serial.println("0 "+String(distance*100)+" 200"); // LowerLimit Reading UpperLimit
                        Sleep(RNG_DELAY_MS - 10);  // start couple of ms earlier
                    }
                }
                else {
                    /* Clear RX error/timeout events in the DW IC
                     * status register. */
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
                }
                
            }
        }else 
        {
            /* Clear RX error/timeout events in the DW IC status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        }

/*--------------------------BLUETOOTH DISTANCE SEND IF UNDER 100---------------------------*/
      // Check if connected
      if((distance * 100) <= 100){
        if (SerialBT.connected()) {
          // Send command that we are almost to the anchor.
          Serial.println("Connected and sending; Distance less than 100");
          SerialBT.println(String("0xCC"));
          closeRange = 0xCC;
          delay(1000);
        }else{
          // Send command that we are almost to the anchor.
          Serial.println("Connected and sending; Distance greater than 100");
          SerialBT.println("0xEE");
          closeRange = 0xEE;
          delay(1000);
      }
        // Receive responses from Slave
        if (SerialBT.available()) {
          Serial.write(SerialBT.read());
        }
      }
/*--------------------------------------------------------------------------------*/
/*--------------------------VL530X RANGING MEASUREMENTS---------------------------*/
      //I2C Pins: IO22 = SCL, IO21 = SDA
      VL53L0X_RangingMeasurementData_t measure;
        
      Serial.print("Reading a measurement... ");
      lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

      if (measure.RangeStatus != 4) {  // phase failures have incorrect data
        Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
        vlxMeasurement = measure.RangeMilliMeter; //VLX Measurement (From Ground)
        vlxMeasurement_Status = 0xAA;
      } else {
        Serial.println(" out of range ");
        vlxMeasurement_Status = 0xFF;
      }
      delay(100);
/*--------------------------------------------------------------------------------*/
/*--------------------------UART SEND TO PYNQ MEASUREMENTS -- DataPacket ---------------------------*/
      distance = distance * 100; //Distance measurement from UWB
      pynqDataPacket[0] = START_BYTE; //Start byte 0xAA to show we are starting transmission
      memcpy(&pynqDataPacket[1], &distance, sizeof(float)); //UWB distance float values into the packet byte array.4 bytes
      pynqDataPacket[5] = START_BYTE; //Success byte
      memcpy(&pynqDataPacket[6], &vlxMeasurement, sizeof(float)); //VlxMeasurement float values into packet byte array
      memcpy(&pynqDataPacket[10], &vlxMeasurement_Status, sizeof(uint8_t)); //Status of float measurement. 0xAA is good, 0xFF is bad.
      memcpy(&pynqDataPacket[11], &closeRange, sizeof(uint8_t)); //Status of closeness measurement. 0xCC is good, 0xEE is bad.
      pynqDataPacket[12] = END_BYTE; //0xBB shows we are ending

      Serial.println("Sending Data Over UART ");
      /* PACKET EXAMPLE: 68.48 1.00 69.69 */
      SerialPort.write(pynqDataPacket, sizeof(pynqDataPacket));
/*--------------------------------------------------------------------------------*/
    }
  }
