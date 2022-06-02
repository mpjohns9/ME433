#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include<stdio.h>

// DEVCFG0
#pragma config DEBUG = OFF // disable debugging
#pragma config JTAGEN = OFF // disable jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF // disable flash write protect
#pragma config BWP = OFF // disable boot write protect
#pragma config CP = OFF // disable code protect

// DEVCFG1
#pragma config FNOSC = FRCPLL // use fast frc oscillator with pll
#pragma config FSOSCEN = OFF // disable secondary oscillator
#pragma config IESO = OFF // disable switching clocks
#pragma config POSCMOD = OFF // primary osc disabled
#pragma config OSCIOFNC = OFF // disable clock output
#pragma config FPBDIV = DIV_1 // divide sysclk freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD // disable clock switch and FSCM
#pragma config WDTPS = PS1048576 // use largest wdt value
#pragma config WINDIS = OFF // use non-window mode wdt
#pragma config FWDTEN = OFF // wdt disabled
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%

// DEVCFG2 - get the sysclk clock to 48MHz from the 8MHz fast rc internal oscillator
#pragma config FPLLIDIV = DIV_2 // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = MUL_24 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = DIV_2 // divide clock after FPLLMUL to get 48MHz

// DEVCFG3
#pragma config USERID = 00000000 // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = OFF // allow multiple reconfigurations
#pragma config IOL1WAY = OFF // allow multiple reconfigurations

#define DESIRED_BAUD 230400
#define SYS_FREQ 48000000ul

#define I2C_WRITE 0b01000000
#define I2C_READ  0b01000001

#define IMU_ADDR 0x68  // I2C hardware address of MPU-6050
#define IMU_ARRAY_LEN 14 // 14 contiguous registers from ACCEL_XOUT_H to GYRO_ZOUT_L

// register addresses
// TODO: config addresses
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C
// TODO: any other config registers I need to use?

// sensor data registers:
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define TEMP_OUT_H   0x41
#define TEMP_OUT_L   0x42
#define GYRO_XOUT_H  0x43
#define GYRO_XOUT_L  0x44
#define GYRO_YOUT_H  0x45
#define GYRO_YOUT_L  0x46
#define GYRO_ZOUT_H  0x47
#define GYRO_ZOUT_L  0x48
#define WHO_AM_I     0x75


void ReadUART1(char * message, int maxLength);
void WriteUART1(const char * string);
void UART1_Startup();


void write_pin(unsigned char reg, unsigned char value);
unsigned char read_pin(unsigned char reg);
void i2c_master_setup(void);
void i2c_master_start(void);
void i2c_master_restart(void);
void i2c_master_send(unsigned char byte);
unsigned char i2c_master_recv(void);
void i2c_master_ack(int val);
void i2c_master_stop(void);

void init_mpu6050(void);
uint8_t whoami(void);
void burst_read_mpu6050(uint8_t * data);

int16_t get_xXL(uint8_t * data);
int16_t get_yXL(uint8_t * data);
int16_t get_zXL(uint8_t * data);
int16_t get_temp(uint8_t * data);
int16_t get_xG(uint8_t * data);
int16_t get_yG(uint8_t * data);
int16_t get_zG(uint8_t * data);
float conv_xXL(uint8_t * data);
float conv_yXL(uint8_t * data);
float conv_zXL(uint8_t * data);
float conv_xG(uint8_t * data);
float conv_yG(uint8_t * data);
float conv_zG(uint8_t * data);
float conv_temp(uint8_t * data);

void burst_read_I2C1(uint8_t dev_addr,
                     uint8_t start_reg_addr,
                     uint8_t * data,
                     uint8_t data_len);

// write one byte (data) from a register (reg_addr) of a device (dev_addr):
void write_byte_I2C1(uint8_t dev_addr,
                     uint8_t reg_addr,
                     uint8_t data);

uint8_t read_byte_I2C1(uint8_t dev_addr, uint8_t reg_addr);

void blink();

int main() {

    __builtin_disable_interrupts(); // disable interrupts while initializing things

    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;

    // do your TRIS and LAT commands here
    TRISAbits.TRISA4 = 0;
    TRISBbits.TRISB4 = 1;

    PORTAbits.RA4 = 0;
    
    LATAbits.LATA4 = 0;
    
    U1RXRbits.U1RXR = 0b0000; // Set A2 to U1RX
    RPB3Rbits.RPB3R = 0b0001; // Set B3 to U1TX
    
    // turn on UART1 without an interrupt
    U1MODEbits.BRGH = 0; // set baud to NU32_DESIRED_BAUD
    U1BRG = ((SYS_FREQ / DESIRED_BAUD) / 16) - 1;

    // 8 bit, no parity bit, and 1 stop bit (8N1 setup)
    U1MODEbits.PDSEL = 0;
    U1MODEbits.STSEL = 0;

    // configure TX & RX pins as output & input pins
    U1STAbits.UTXEN = 1;
    U1STAbits.URXEN = 1;
    
    // configure hardware flow control using RTS and CTS
    U1MODEbits.UEN = 2;

    // enable the uart
    U1MODEbits.ON = 1;
    
    i2c_master_setup();
    
    UART1_Startup();
    
    __builtin_enable_interrupts();
    
    // init the imu
    init_mpu6050();
    
    char m_in[100]; // char array for uart data coming in
    char m_out[200]; // char array for uart data going out
    int i;
    #define NUM_DATA_PNTS 300 // how many data points to collect at 100Hz
    float ax[NUM_DATA_PNTS], ay[NUM_DATA_PNTS], az[NUM_DATA_PNTS], gx[NUM_DATA_PNTS], gy[NUM_DATA_PNTS], gz[NUM_DATA_PNTS], temp[NUM_DATA_PNTS];
    
    //sprintf(m_out,"MPU-6050 WHO_AM_I: %X\r\n",whoami());
    //WriteUART1(m_out);
    char who = whoami(); // ask if the imu is there
    if (who != 0x68){
        // if the imu is not there, get stuck here forever
        while(1){
            LATAbits.LATA4 = 1;
        }
    }
    
    char IMU_buf[IMU_ARRAY_LEN]; // raw 8 bit array for imu data

    while (1) {
        blink();
        
        ReadUART1(m_in,100); // wait for a newline
        // don't actually have to use what is in m
        
        // collect data
        for (i=0; i<NUM_DATA_PNTS; i++){
            _CP0_SET_COUNT(0);
            // read IMU
            burst_read_mpu6050(IMU_buf);
            ax[i] = conv_xXL(IMU_buf);
            ay[i] = conv_yXL(IMU_buf);
            az[i] = conv_zXL(IMU_buf);
            gx[i] = conv_xG(IMU_buf);
            gy[i] = conv_yG(IMU_buf);
            gz[i] = conv_zG(IMU_buf);
            temp[i] = conv_temp(IMU_buf);
            
            while(_CP0_GET_COUNT()<24000000/2/100){}
        }
        
        // print data
        for (i=0; i<NUM_DATA_PNTS; i++){
            sprintf(m_out,"%d %f %f %f %f %f %f %f\r\n",NUM_DATA_PNTS-i,ax[i],ay[i],az[i],gx[i],gy[i],gz[i],temp[i]);
            WriteUART1(m_out);
        }
        
    }
}

// Read from UART1
// block other functions until you get a '\r' or '\n'
// send the pointer to your char array and the number of elements in the array
void ReadUART1(char * message, int maxLength) {
  char data = 0;
  int complete = 0, num_bytes = 0;
  // loop until you get a '\r' or '\n'
  while (!complete) {
    if (U1STAbits.URXDA) { // if data is available
      data = U1RXREG;      // read the data
      if ((data == '\n') || (data == '\r')) {
        complete = 1;
      } else {
        message[num_bytes] = data;
        ++num_bytes;
        // roll over if the array is too small
        if (num_bytes >= maxLength) {
          num_bytes = 0;
        }
      }
    }
  }
  // end the string
  message[num_bytes] = '\0';
}

// Write a character array using UART3
void WriteUART1(const char * string) {
  while (*string != '\0') {
    while (U1STAbits.UTXBF) {
      ; // wait until tx buffer isn't full
    }
    U1TXREG = *string;
    ++string;
  }
}

void UART1_Startup() {
  // disable interrupts
  __builtin_disable_interrupts();

  // turn on UART1 without an interrupt
  U1MODEbits.BRGH = 0; // set baud to PIC32_DESIRED_BAUD
  U1BRG = ((SYS_FREQ / DESIRED_BAUD) / 16) - 1;

  // 8 bit, no parity bit, and 1 stop bit (8N1 setup)
  U1MODEbits.PDSEL = 0;
  U1MODEbits.STSEL = 0;

  // configure TX & RX pins as output & input pins
  U1STAbits.UTXEN = 1;
  U1STAbits.URXEN = 1;

  // enable the uart
  U1MODEbits.ON = 1;

  __builtin_enable_interrupts();
}

void blink(){
    LATAbits.LATA4 = 1;
    _CP0_SET_COUNT(0);
    while(_CP0_GET_COUNT()<24000000/2/20){}
    LATAbits.LATA4 = 0;
    _CP0_SET_COUNT(0);
    while(_CP0_GET_COUNT()<24000000/2/20){}
}

void write_pin(unsigned char reg, unsigned char value)
{
    i2c_master_start();
    i2c_master_send(I2C_WRITE);
    i2c_master_send(reg);
    i2c_master_send(value);
    i2c_master_stop();
}

unsigned char read_pin(unsigned char reg)
{
    i2c_master_start();
    i2c_master_send(I2C_WRITE);
    i2c_master_send(reg);
    i2c_master_restart();
    i2c_master_send(I2C_READ);
    unsigned char r = i2c_master_recv();
    i2c_master_ack(1);
    i2c_master_stop();
    return r;
}

void i2c_master_setup(void) {
    // using a large BRG to see it on the nScope, make it smaller after verifying that code works
    // look up TPGD in the datasheet
    I2C1BRG = 1000; // I2CBRG = [1/(2*Fsck) - TPGD]*Pblck - 2 (TPGD is the Pulse Gobbler Delay)
    I2C1CONbits.ON = 1; // turn on the I2C1 module
}

void i2c_master_start(void) {
    I2C1CONbits.SEN = 1; // send the start bit
    while (I2C1CONbits.SEN) {
        ;
    } // wait for the start bit to be sent
}

void i2c_master_restart(void) {
    I2C1CONbits.RSEN = 1; // send a restart 
    while (I2C1CONbits.RSEN) {
        ;
    } // wait for the restart to clear
}

void i2c_master_send(unsigned char byte) { // send a byte to slave
    I2C1TRN = byte; // if an address, bit 0 = 0 for write, 1 for read
    while (I2C1STATbits.TRSTAT) {
        ;
    } // wait for the transmission to finish
    if (I2C1STATbits.ACKSTAT) { // if this is high, slave has not acknowledged
        // ("I2C1 Master: failed to receive ACK\r\n");
        while(1){} // get stuck here if the chip does not ACK back
    }
}

unsigned char i2c_master_recv(void) { // receive a byte from the slave
    I2C1CONbits.RCEN = 1; // start receiving data
    while (!I2C1STATbits.RBF) {
        ;
    } // wait to receive the data
    return I2C1RCV; // read and return the data
}

void i2c_master_ack(int val) { // sends ACK = 0 (slave should send another byte)
    // or NACK = 1 (no more bytes requested from slave)
    I2C1CONbits.ACKDT = val; // store ACK/NACK in ACKDT
    I2C1CONbits.ACKEN = 1; // send ACKDT
    while (I2C1CONbits.ACKEN) {
        ;
    } // wait for ACK/NACK to be sent
}

void i2c_master_stop(void) { // send a STOP:
    I2C1CONbits.PEN = 1; // comm is complete and master relinquishes bus
    while (I2C1CONbits.PEN) {
        ;
    } // wait for STOP to complete
}

void init_mpu6050(void) {
    i2c_master_setup(); // initialize I2C1 (used for comm w/ IMU)
    //write_UART1("Initialized I2C1.\r\n");
    
    // 1. Configure IMU to get clock signal from internal 8 MHz oscillator:
    //  write to MPU-6050 PWR_MGMT_1 register,
    //  clear bit 6 (defaults to 1) to wake IMU from sleep (logic is awake but
    //  MEMS sensor stuff is asleep)
    write_byte_I2C1(IMU_ADDR,PWR_MGMT_1,0x00);
    //delay(); // delay a bit just to be safe. TODO: remove?
    
    // 2. TODO: initialize CONFIG register
    //  bits [2:0]: configure digital low pass filter
    //  bits [5:3]: configure external sync (leave disabled)
    //  bits [7:6]: reserved
    
    // 3. Initialize ACCEL_CONFIG (accelerometer) register
    //  bits [2:0]: reserved
    //  bits [4:3]: configure full scale range; TODO: update
    //      0b00: +/- 2g
    //      0b01: +/- 4g
    //      0b10: +/- 8g
    //      0b11: +/- 16g -- use this for now
    //  bits [7:5]: configure accelerometer self-test (leave disabled)
    write_byte_I2C1(IMU_ADDR,ACCEL_CONFIG,0x00); // 0x0: +/- 2g, no self-tests

    // 4. Initialize MPU-6050 GYRO_CONFIG (gyroscope) register
    //  bits [2:0]: reserved
    //  bits [4:3]: configure full scale range; TODO: update
    //      0b00: +/-  250 dps
    //      0b01: +/-  500 dps
    //      0b10: +/- 1000 dps
    //      0b11: +/- 2000 dps -- use this for now
    //  bits [7:5]: configure gyroscope self-test (leave disabled)
    write_byte_I2C1(IMU_ADDR,GYRO_CONFIG,0x0C); // 0x0C: +/- 2000 dps, no self-tests

    // TODO: configure for sequential reading
}

// Verify the identity of the MPU-6050.
// If MPU-6050 is functioning and communications are working,
// this function will return 0x68
uint8_t whoami(void) {
    return read_byte_I2C1(IMU_ADDR, WHO_AM_I);
}

// Burst read from MPU-6050, from ACCEL_XOUT_H reg through GYRO_ZOUT_L reg.
// Raw data from IMU registers is stored in "data" - an array passed to this
// function by reference.
// The number of registers this function reads from is defined by IMU_ARRAY_LEN,
// and this function does not check to see if the length of "data" equals
// IMU_ARRAY_LEN or not.
// This function is basically just a wrapper for burst_read_I2C1(), defined in
// i2c_noint.{c,h}.
void burst_read_mpu6050(uint8_t * data) {
    burst_read_I2C1(IMU_ADDR,ACCEL_XOUT_H,data,IMU_ARRAY_LEN);
}

/*******************************************************************************
 Functions to combine 8-bit register pairs (each uint8_t) into int16_t's:
 ******************************************************************************/
int16_t get_xXL(uint8_t * data) { // convert x-acceleration LSB and MSB to int16_t
    return data[0]<<8 | data[1];
}

int16_t get_yXL(uint8_t * data) { // convert y-acceleration LSB and MSB to int16_t
    return data[2]<<8 | data[3];
}

int16_t get_zXL(uint8_t * data) { // convert z-acceleration LSB and MSB to int16_t
    return data[4]<<8 | data[5];
}

int16_t get_temp(uint8_t * data) { // convert temperature LSB and MSB to int16_t
    return data[6]<<8 | data[7];
}

int16_t get_xG(uint8_t * data) { // convert x-gyro LSB and MSB to int16_t
    return data[8]<<8 | data[9];
}

int16_t get_yG(uint8_t * data) { // convert y-gyro LSB and MSB to int16_t
    return data[10]<<8 | data[11];
}

int16_t get_zG(uint8_t * data) { // convert z-gyro LSB and MSB to int16_t
    return data[12]<<8 | data[13];
}

// TODO: test these functions!!!
/*
 * Functions to convert int16_t representation of 16-bit IMU data (acceleration
 * in x, y, and z; temperature; gyro rates) to float representation
 */
float conv_xXL(uint8_t * data) { // convert x-acceleration to float (g's)
    return (get_xXL(data))*0.000061;
}

float conv_yXL(uint8_t * data) { // convert y-acceleration to float (g's)
    return (get_yXL(data))*0.000061;
}

float conv_zXL(uint8_t * data) { // convert z-acceleration to float (g's)
    return (get_zXL(data))*0.000061;
}

float conv_xG(uint8_t * data) { // convert x-gyro rate to dps
    return (get_xG(data))*0.007630;
}

float conv_yG(uint8_t * data) { // convert y-gyro rate to dps
    return (get_yG(data))*0.007630;
}

float conv_zG(uint8_t * data) { // convert z-gyro rate to dps
    return (get_zG(data))*0.007630;
}

float conv_temp(uint8_t * data) { // convert int16_t temperature signed short to float (Celsius)
    return (get_temp(data)/340.00) + 36.53;
}

// i2c functions
// read one byte from a register (reg_addr) of a device (dev_addr):
uint8_t read_byte_I2C1(uint8_t dev_addr,
                       uint8_t reg_addr) {
    uint8_t answer;
    i2c_master_start();
    i2c_master_send(dev_addr << 1); // hardware address and write bit
    i2c_master_send(reg_addr);  // WHO_AM_I register: 0x0F
    i2c_master_restart(); // this line is REALLY important!
    i2c_master_send((dev_addr << 1) | 1); // hardware address and read bit
    answer = i2c_master_recv(); // receive a byte from the slave. Should be 0x69 = 105d
    i2c_master_ack(1); // send NACK to slave
    i2c_master_stop();
    return answer;
}

// burst read from device (dev_addr), beginning at specified register by
// start_reg_addr:
void burst_read_I2C1(uint8_t dev_addr,
                     uint8_t start_reg_addr,
                     uint8_t * data,
                     uint8_t data_len) {
    char i;
    i2c_master_start();
    i2c_master_send((dev_addr << 1)); // hardware address and write bit
    i2c_master_send(start_reg_addr);  // starting register. "register" has some other meaning in C, so I called it "registerrr" instead.
    i2c_master_restart(); // this line is REALLY important!
    i2c_master_send((dev_addr << 1) | 1); // hardware address and read bit
    for (i = 0; i < data_len; i++) {
        data[i] = i2c_master_recv(); // receive a byte from the slave. Should be 0x69 = 105d
        if (i==(data_len-1)) {
            i2c_master_ack(1); // send NACK to slave to stop reading
        }
        else {
            i2c_master_ack(0); // send ACK to slave to continue reading
        }
    }
    i2c_master_stop();
}

// write one byte (data) from a register (reg_addr) of a device (dev_addr):
void write_byte_I2C1(uint8_t dev_addr,
                     uint8_t reg_addr,
                     uint8_t data) {
    i2c_master_start();             // START bit
    i2c_master_send(dev_addr << 1); // hardware address; RW (lsb) = 0, indicates write
    i2c_master_send(reg_addr);      // specify register to write to
    i2c_master_send(data);          // specify data to write to reg_addr
    i2c_master_stop();              // STOP bit
}
