#include "Adafruit_VL53L0X.h"
#include "dw3000.h"
#include "BluetoothSerial.h"
#include <HardwareSerial.h>

#define APP_NAME "DS TWR RESP v1.0"

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

/* Default antenna delay values for 64 MHz PRF. */
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

/* Frames used in the ranging process */
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
 */
extern dwt_txconfig_t txconfig_options;
/*--------------------------BLUETOOTH CONFIGURATION---------------------------*/
BluetoothSerial SerialBT;
int i = 0;

String MACadd = "AA:BB:CC:11:22:33"; //This can be configurable - Simple selection for now
uint8_t address[6]  = {0xAA, 0xBB, 0xCC, 0x11, 0x22, 0x33}; 

String name = "ESP32_Transmitter"; //Name of bluetooth, confirm this is the same for comm BT devices
char *pin = "1234"; //Standard pin would be provided by default for now.
bool connected;
/*--------------------------VL53L TOF SENSOR CONFIGURATION-----------------------*/
Adafruit_VL53L0X lox = Adafruit_VL53L0X(); //Vendor library - VL53L0X does NOT have vendor documentation :(
float vlxMeasurement = 0; 
uint8_t vlxMeasurement_Status = 0xAA;
/*--------------------------UART SEND CONFIGURATION-----------------------*/
#define START_BYTE 0xAA //pynqDataPacket start byte
#define END_BYTE 0xBB //pynqDataPacket end byte

uint8_t closeRange = 0xEE; //Variable used to identify closeness
uint8_t pynqDataPacket[13]; //Data packet sent to Pynq over UART
HardwareSerial SerialPort(2); // USE OF UART1 - Both UARTs used

void setup() 
{
/*--------------------------UART SEND CONFIGURATION-----------------------*/
  SerialPort.begin(115200, SERIAL_8N1, 16, 17); 
  UART_init();
/*--------------------------UWB CONFIGURATION-----------------------------*/
  test_run_info((unsigned char *)APP_NAME);
/*--------------------------SPI CONFIGURATION-----------------------------*/
  spiBegin(PIN_IRQ, PIN_RST);
  spiSelect(PIN_SS);
/*--------------------------BLUETOOTH CONNECT---------------------------*/
  SerialBT.begin("ESP32testm", true); 
  Serial.println("The device started in master mode, make sure remote BT device is on!");
  connected = SerialBT.connect(name);

  if(connected) { //Check if able to connect successfully
      Serial.println("Connected Succesfully!");}
  else{
      while(!SerialBT.connected(10000)) {
        Serial.println("Failed to connect. Make sure remote device is available and in range, then restart app."); 
      }
  }

  if (SerialBT.disconnect()) {
    Serial.println("Disconnected Succesfully!");
  }
  SerialBT.connect();
  Sleep(2); //Sleep to allow BT connections before UWB setup.
  /*-------------------UWB CONFIGURATION CONTINUED---------------------------------*/
  while (!dwt_checkidlerc()) { //Need to make sure DW IC is in IDLE_RC before proceeding 
    UART_puts("IDLE FAILED\r\n");
    while (1) ;
  };

  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
    while (1) { /* spin */ }; //Error with INIT
  }

  if (dwt_configure(&config)) { // Configure DW IC
    while (1) { /* spin */ }; //Error with INIT
  }
    
    dwt_configuretxrf(&txconfig_options); //Configure the TX spectrum parameters
    dwt_setrxantennadelay(RX_ANT_DLY); //Apply default antenna delay value
    dwt_settxantennadelay(TX_ANT_DLY);
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE); //Enable TX/RX states output on GPIOs 5 and 6 to help debug,
    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);
/*--------------------------VL53L0X INIT-----------------------------*/
  Serial.begin(115200);

  while (! Serial) { // Wait until serial port opens for native USB devices
    delay(1);
  }
  Serial.println("Adafruit VL53L0X test");

  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }

}    

    void loop (){
      while (1){
        dwt_setpreambledetecttimeout(0);
        dwt_setrxtimeout(0); // Clear reception timeout to start next ranging process
        dwt_rxenable(DWT_START_RX_IMMEDIATE); //Activate reception immediately.

        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK |SYS_STATUS_ALL_RX_TO |SYS_STATUS_ALL_RX_ERR))) // Poll for reception of a frame or error/timeout
        { /* spin */ };
        if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {

            
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK); // Clear good RX frame event in the DW IC status register.
            uint32_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & FRAME_LEN_MAX_EX; // A frame has been received, read it into the local buffer.
            if (frame_len <= RX_BUF_LEN) {
                dwt_readrxdata(rx_buffer, frame_len, 0);
            }

            rx_buffer[ALL_MSG_SN_IDX] = 0;
            if (memcmp(rx_buffer, rx_poll_msg, ALL_MSG_COMMON_LEN) == 0) { //Check that the frame is a poll sent by "DS TWR initiator" example.
                uint32_t resp_tx_time;
                poll_rx_ts = get_rx_timestamp_u64(); //Retrieve poll reception timestamp.
                resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8; //Set send time for response
                dwt_setdelayedtrxtime(resp_tx_time);
                dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS); //Set expected delay and timeout for final message reception
                dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);
                dwt_setpreambledetecttimeout(PRE_TIMEOUT); // Set preamble timeout for expected frames
                tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb; // Write and send the response message
                dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0); /* Zero offset in TX buffer. */
                dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1); /* Zero offset in TX buffer, ranging. */
                int ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);
                if (ret == DWT_ERROR) {
                    continue;
                }

                while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & //Poll for reception of expected "final" frame or error/timeout.
                                                       (SYS_STATUS_RXFCG_BIT_MASK |
                                                        SYS_STATUS_ALL_RX_TO |
                                                        SYS_STATUS_ALL_RX_ERR)))
                { /* spin */ };
                frame_seq_nb++; // Increment frame sequence number after transmission of the

                if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_TXFRS_BIT_MASK); //Clear good RX frame event and TX frame sent in the DW IC status register. 
                    frame_len = dwt_read32bitreg(RX_FINFO_ID) & FRAME_LEN_MAX_EX; // A frame has been received, read it into the local buffer.

                    if (frame_len <= RX_BUF_LEN) {
                        dwt_readrxdata(rx_buffer, frame_len, 0);
                    }
                    rx_buffer[ALL_MSG_SN_IDX] = 0;
                    if (memcmp(rx_buffer, rx_final_msg, ALL_MSG_COMMON_LEN) == 0) {
                        uint32_t poll_tx_ts, resp_rx_ts, final_tx_ts;
                        uint32_t poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
                        double Ra, Rb, Da, Db;
                        int64_t tof_dtu;

                        resp_tx_ts = get_tx_timestamp_u64();  // Retrieve response transmission and final reception timestamps.
                        final_rx_ts = get_rx_timestamp_u64();

                        /* Get timestamps embedded in the final message. */
                        final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts);
                        final_msg_get_ts(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX], &resp_rx_ts);
                        final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts);

                        /* Compute time of flight.*/
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
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR); //Clear RX error/timeout events in the DW IC status register.
                }
                
            }
        }else 
        {
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR); //Clear RX error/timeout events in the DW IC status register.
        }

      /*--------------------------BLUETOOTH DISTANCE SEND IF UNDER 100---------------------------*/
      if((distance * 100) <= 100){ // Check if UWB distance is under 100 mm
        if (SerialBT.connected()) {
          Serial.println("Connected and sending; Distance less than 100"); //DEBUG
          SerialBT.println(String("0xCC")); //Send the 'closeness' command to anchor.
          closeRange = 0xCC;
          delay(1000);
        }else{
          Serial.println("Connected and sending; Distance greater than 100"); // DEBUG
          SerialBT.println("0xEE"); //Send that we aren't within threshold to the anchor.
          closeRange = 0xEE;
          delay(1000);
        }
        if (SerialBT.available()) { 
            Serial.write(SerialBT.read()); //Receive responses from anchor
        }
      }
      /*--------------------------VL530X RANGING MEASUREMENTS---------------------------*/
      /*I2C Pins: IO22 = SCL, IO21 = SDA*/
      VL53L0X_RangingMeasurementData_t measure;
      Serial.print("Reading a measurement... "); //DEBUG
      lox.rangingTest(&measure, false); // False: No DEBUG, True: DEBUG

      if (measure.RangeStatus != 4){  // Check for Phase Failures -- Should be ignored.
        Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
        vlxMeasurement = measure.RangeMilliMeter; //VLX Measurement
        vlxMeasurement_Status = 0xAA; //Tell anchor we are within measuring range, otherwise we get nonsense.
      } else {
        Serial.println(" Out of Range ");
        vlxMeasurement_Status = 0xFF; //Tell anchor we are not within measuring range.
      }
      delay(100);
    /*--------------------------UART SEND TO PYNQ MEASUREMENTS -- DataPacket ---------------------------*/
      distance = distance * 100; //Distance measurement from UWB - convert to proper measurement.
      pynqDataPacket[0] = START_BYTE; //Start byte 0xAA to show we are starting transmission
      memcpy(&pynqDataPacket[1], &distance, sizeof(float)); //UWB distance float values into the packet byte array -- 4 BYTES
      pynqDataPacket[5] = START_BYTE; //SUCCESS BYTE (same as start byte)
      memcpy(&pynqDataPacket[6], &vlxMeasurement, sizeof(float)); //VlxMeasurement float values into packet byte array -- 4 BYTES
      memcpy(&pynqDataPacket[10], &vlxMeasurement_Status, sizeof(uint8_t)); //Status of float measurement -- 0xAA == GOOD, 0xFF == BAD.
      memcpy(&pynqDataPacket[11], &closeRange, sizeof(uint8_t)); //Status of closeness measurement. 0xCC == GOOD, 0xEE == BAD.
      pynqDataPacket[12] = END_BYTE; //0xBB shows we are ending

      Serial.println("Sending Data Over UART: "); //DEBUG
      SerialPort.write(pynqDataPacket, sizeof(pynqDataPacket)); //Write to UART to Pynq.
    }
  }
