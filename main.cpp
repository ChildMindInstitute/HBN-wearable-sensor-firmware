
#include "mbed.h"
#include "mbed_i2c.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "nrf51.h"
#include "nrf51_bitfields.h"

#include "BLE.h"
#include "DFUService.h"
#include "UARTService.h"


#define LOG(...)    { pc.printf(__VA_ARGS__); }

#define LED_GREEN       p21
#define LED_RED         p22
#define LED_BLUE        p23
#define BUTTON_PIN      p17
#define BATTERY_PIN     p1
#define MICROPHONE_PIN  p5

#define MPU6050_SDA     p12
#define MPU6050_SCL     p13

#define UART_TX         p9 
#define UART_RX         p11 
#define UART_CTS    p8
#define UART_RTS    p10

/* Starting MPU6050sampling rate. */
#define DEFAULT_MPU_HZ  (100)

/* Master Data Polling/Sampling Speed in second intervals --> 1/Hz */
#define SENSOR_POLL_RATE_SECONDS  (0.2)  // 5Hz

DigitalOut blue(LED_BLUE);
DigitalOut green(LED_GREEN);
DigitalOut red(LED_RED);

InterruptIn button(BUTTON_PIN);
AnalogIn    battery(BATTERY_PIN);
//AnalogIn    microphone(MICROPHONE_PIN);

Serial pc(UART_TX, UART_RX);

/* Toggle both wired and bluetooth serial out UART logging */
bool logging = true; 

InterruptIn motion_probe(p14);

int read_none_count = 0;

BLEDevice  ble;
UARTService *uartServicePtr;

volatile bool bleIsConnected = false;
volatile uint8_t tick_event = 0;
volatile uint8_t motion_event = 0;
static signed char board_orientation[9] = {
    1, 0, 0,
    0, 1, 0,
    0, 0, 1
};
//for pedometer
unsigned long stepCount = 0;
unsigned long stepTime = 0;
unsigned long lastStepCount = 0;


void check_i2c_bus(void);
unsigned short inv_orientation_matrix_to_scalar( const signed char *mtx);


void connectionCallback(const Gap::ConnectionCallbackParams_t *params)
{
    LOG("Connected!\n");
    bleIsConnected = true;
}

void disconnectionCallback(const Gap::DisconnectionCallbackParams_t *cbParams)
{
    LOG("Disconnected!\n");
    LOG("Restarting the advertising process\n");
    ble.startAdvertising();
    bleIsConnected = false;
}

/* ----------- Microphone Class ------------ */
class microphone
{
public :
    microphone(PinName pin);
    float read();
    operator float ();
private :
    AnalogIn _pin;
};
microphone::microphone (PinName pin):
    _pin(pin)
{
}
float microphone::read()
{
    return _pin.read();
}
inline microphone::operator float ()
{
    return _pin.read();
}
// declare microphone object/class member
microphone mic(MICROPHONE_PIN);


/* --------- PRIMARY SENSOR POLL FUNCTION --------- */
void tick(void)
{
    static uint32_t count = 0;
    
    LOG("%d\r\n", count++);
    green = !green;
    
    unsigned long sensor_timestamp;
    short gyro[3], accel[3], sensors;
    short accel_comp[3];
    long quat[4];
    unsigned char more = 1;
    

    
  //  float sample; //audio sample from mic
    float ambientSound;
    
 //   int steps;
    
    /* Sample microphone for ambient noise */
    /* read in sample value using AC coupled input option averaging four samples with op amp for gain */
   // sample = (mic + mic + mic + mic)/4.0;
    // ambientSound = 0.5 + (sample -0.5);     //subtract the DC bias (1.65V) and add gain for speaker
    ambientSound = abs(( mic  - (0.67/3.3)))*500.0;
 
      printf("NOISE: ");
      char array1[15];
      sprintf(array1, "%f", ambientSound );
      printf(array1);
      printf(" \n");
   // LOG("NOISE: %d\n", noise);
    
    /* get number of steps since boot from DMP pedometer */
    int calldmp; //dummy
    calldmp = dmp_get_pedometer_step_count(&stepCount);
    calldmp = dmp_get_pedometer_walk_time(&stepTime);
     printf("STEPS: ");
      char array2[15];
      sprintf(array2, "%ul", stepCount );
      printf(array2);
      printf(" \n");
    
    /* Wake up all MPU6050 sensors & enter streaming mode. */
    dmp_set_interrupt_mode(DMP_INT_CONTINUOUS);  //Switch to streaming
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    /* Push both gyro and accel data into the FIFO. */
  //  mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    
    dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);
    
    if (sensors & INV_XYZ_GYRO) {
     //   LOG("GYRO: %d, %d, %d\n", gyro[0], gyro[1], gyro[2]);
    }
    if (sensors & INV_XYZ_ACCEL) {
        //compute angular position 
        accel_comp[0] = (180/3.141592) * atan( accel[0] / pow( (accel[1]*accel[1]) + (accel[2]*accel[2]) , 0.5) ); 
        accel_comp[1] = (180/3.141592) * atan( accel[1] / pow( (accel[0]*accel[0]) + (accel[2]*accel[2]) , 0.5) ); 
        accel_comp[2] = (180/3.141592) * atan( pow( (accel[1]*accel[1]) + (accel[0]*accel[0]) , 0.5) / accel[2] );
        
        //log angular position 
     //   LOG("ACC: %d, %d, %d\n", accel[0], accel[1], accel[2]); 
        LOG("ACC COMP: %d, %d, %d\n", accel_comp[0], accel_comp[1], accel_comp[2]);
    }
    
    dmp_set_interrupt_mode(DMP_INT_GESTURE);  //back to interrupt/pedometer mode
}

void detect(void)
{
    LOG("Button pressed\n");  
    blue = !blue;
    /* toggle wired and bluetooth UART serial logging */
    logging = !logging;
}

void motion_interrupt_handle(void)
{
    motion_event = 1;
}

void tap_cb(unsigned char direction, unsigned char count)
{
    LOG("Tap motion detected\n");
}

void android_orient_cb(unsigned char orientation)
{
    LOG("Oriention changed\n");
}


int main(void)
{
    blue  = 1;
    green = 1;
    red   = 1;

    pc.baud(115200);
    
    wait(1);
    
    LOG("---- Seeed Tiny BLE ----\n");
    
    mbed_i2c_clear(MPU6050_SDA, MPU6050_SCL);
    mbed_i2c_init(MPU6050_SDA, MPU6050_SCL);
    

    if (mpu_init(0)) {
        LOG("failed to initialize mpu6050\r\n");
    }
    
    /* Get/set hardware configuration. Start gyro. */
    /* Wake up all sensors. */
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    /* Push both gyro and accel data into the FIFO. */
    mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_set_sample_rate(DEFAULT_MPU_HZ);
    
    /* Read back configuration in case it was set improperly. */
    unsigned char accel_fsr;
    unsigned short gyro_rate, gyro_fsr;
    mpu_get_sample_rate(&gyro_rate);
    mpu_get_gyro_fsr(&gyro_fsr);
    mpu_get_accel_fsr(&accel_fsr);
    
    dmp_load_motion_driver_firmware();
    dmp_set_orientation(
        inv_orientation_matrix_to_scalar(board_orientation));
    dmp_register_tap_cb(tap_cb);
    dmp_register_android_orient_cb(android_orient_cb);
    
    uint16_t dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP | DMP_FEATURE_PEDOMETER |
                       DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
                       DMP_FEATURE_GYRO_CAL;
    dmp_enable_feature(dmp_features);
    dmp_set_fifo_rate(DEFAULT_MPU_HZ);
    mpu_set_dmp_state(1);
    
    dmp_set_interrupt_mode(DMP_INT_GESTURE);
    dmp_set_tap_thresh(TAP_XYZ, 50);
    
    dmp_set_pedometer_step_count(stepCount);
    dmp_set_pedometer_walk_time(stepTime);
    
    
    motion_probe.fall(motion_interrupt_handle);


    /* Ticker Controlls Streaming Sensor Polling */
    Ticker ticker;
    ticker.attach(tick, SENSOR_POLL_RATE_SECONDS );


    button.fall(detect);

    LOG("Initialising the nRF51822\n");
    ble.init();
    ble.gap().onDisconnection(disconnectionCallback);
    ble.gap().onConnection(connectionCallback);


    /* setup advertising */
    ble.accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED);
    ble.setAdvertisingType(GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED);
    ble.accumulateAdvertisingPayload(GapAdvertisingData::SHORTENED_LOCAL_NAME,
                                     (const uint8_t *)"smurfs", sizeof("smurfs"));
    ble.accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LIST_128BIT_SERVICE_IDS,
                                     (const uint8_t *)UARTServiceUUID_reversed, sizeof(UARTServiceUUID_reversed));
    DFUService dfu(ble);                                 
    UARTService uartService(ble);
    uartServicePtr = &uartService;
    //uartService.retargetStdout();

    ble.setAdvertisingInterval(160); /* 100ms; in multiples of 0.625ms. */
    ble.gap().startAdvertising();
    
    while (true) {
        if (motion_event) {
            
            unsigned long sensor_timestamp;
            short gyro[3], accel[3], sensors;
            long quat[4];
            unsigned char more = 1;
            
            while (more) {
                /* This function gets new data from the FIFO when the DMP is in
                 * use. The FIFO can contain any combination of gyro, accel,
                 * quaternion, and gesture data. The sensors parameter tells the
                 * caller which data fields were actually populated with new data.
                 * For example, if sensors == (INV_XYZ_GYRO | INV_WXYZ_QUAT), then
                 * the FIFO isn't being filled with accel data.
                 * The driver parses the gesture data to determine if a gesture
                 * event has occurred; on an event, the application will be notified
                 * via a callback (assuming that a callback function was properly
                 * registered). The more parameter is non-zero if there are
                 * leftover packets in the FIFO.
                 */
                dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors,
                              &more);
                
                
                /* Gyro and accel data are written to the FIFO by the DMP in chip
                 * frame and hardware units. This behavior is convenient because it
                 * keeps the gyro and accel outputs of dmp_read_fifo and
                 * mpu_read_fifo consistent.
                 */
                if (sensors & INV_XYZ_GYRO) {
                    LOG("GYRO: %d, %d, %d\n", gyro[0], gyro[1], gyro[2]);
                }
                if (sensors & INV_XYZ_ACCEL) {
                    LOG("ACC: %d, %d, %d\n", accel[0], accel[1], accel[2]);
                }
                
                /* Unlike gyro and accel, quaternions are written to the FIFO in
                 * the body frame, q30. The orientation is set by the scalar passed
                 * to dmp_set_orientation during initialization.
                 */
                if (sensors & INV_WXYZ_QUAT) {
                    LOG("QUAT: %ld, %ld, %ld, %ld\n", quat[0], quat[1], quat[2], quat[3]);
                }
                
                if (sensors) {
                    read_none_count = 0;
                } else {
                    read_none_count++;
                    if (read_none_count > 3) {
                        read_none_count = 0;
                        
                        LOG("I2C may be stuck @ %d\r\n", sensor_timestamp);
                        mbed_i2c_clear(MPU6050_SDA, MPU6050_SCL);
                    }
                }
            }
            
            motion_event = 0;
        } else {
            ble.waitForEvent();
        }
    }
}

/* These next two functions converts the orientation matrix (see
 * gyro_orientation) to a scalar representation for use by the DMP.
 * NOTE: These functions are borrowed from Invensense's MPL.
 */
static inline unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}

