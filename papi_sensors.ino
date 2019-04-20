// < includes >------------------------------------------------------------------------------------

#include <SPI.h>
#include <Wire.h>

#include <TinyGPS.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

#include <SoftwareSerial.h>

#include "SparkFunMPL3115A2.h"

// < defines >-------------------------------------------------------------------------------------

#define D_BMP280
#define D_MPL3115
#define D_GPS
// #define D_DEBUG

#define D_SER_BAUD 57600
#define D_CLB_SAMPLES 120.

// wait time (1000/D_TIM_WAIT) = Hz
#define D_TIM_WAIT 500    // 2 Hz

// < global data >---------------------------------------------------------------------------------

#ifdef D_BMP280
// create an instance of the object
Adafruit_BMP280 g_bmp280;

// pressão nível do mar (QNH) (this should be adjusted to your local forcase)
float g_QNH = 1015;

// bias de altitude
float gf_alt_bmp = 0;
#endif

#ifdef D_MPL3115
// create an instance of the object
MPL3115A2 g_mpl3115;

// bias de altitude
float gf_alt_mpl = 0;
#endif

#ifdef D_GPS
// create an instance of the object
TinyGPS g_gps;

SoftwareSerial g_ss(2, 3);
#endif

// ------------------------------------------------------------------------------------------------
void setup() 
{
    // join i2c bus
    Wire.begin();

    // start serial for output
    Serial.begin(D_SER_BAUD);

    #ifdef D_BMP280
    // BMP 280 init ok ?
    if (!g_bmp280.begin()) 
    {  
        Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
        while (1);

    } // end if
    #endif

    #ifdef D_MPL3115
    // init MPL3115
    setup_MPL3115();
    #endif

    #ifdef D_GPS
    // init GPS connection
    g_ss.begin(4800);
    #endif

    // calibração
    calibra();

} // setup

// ------------------------------------------------------------------------------------------------
void loop() 
{
    #ifdef D_MPL3115
    // altitude calc
    float lf_Px;
    float lf_off_h;
    #endif

    #ifdef D_GPS
    // GPS new data
    bool lv_new_data = false;

    // GPS data
    float lf_lat;
    float lf_lon;

    unsigned long lul_age;
    #endif

    // tempo inicial
    unsigned long lul_ini;
    // elapsed time
    unsigned long lul_elp;

    // get initial time (ms)
    lul_ini = millis();

    // send altitude
    Serial.print("!@ALT#");
    #ifdef D_BMP280
    Serial.print(g_bmp280.readAltitude(g_QNH) - gf_alt_bmp);
    Serial.print("#");
    #endif

    #ifdef D_MPL3115
    // measure altitude above sea level in meters
    lf_Px = 1. - pow(g_mpl3115.readPressure() / 101325, 0.1902632);
    lf_off_h = 60.;

    Serial.print(((44330.77 * lf_Px) + lf_off_h) - gf_alt_mpl);
    Serial.print("#");
    #endif

    Serial.print(millis() / 1000.);
    Serial.println();

    // send pressure
    Serial.print("!@BAR#");
    #ifdef D_BMP280
    // send millibar pressure 
    Serial.print(g_bmp280.readPressure() / 100.);
    Serial.print("#");
    #endif

    #ifdef D_MPL3115
    // send millibar pressure
    Serial.print(g_mpl3115.readPressure() / 100.);
    Serial.print("#");
    #endif

    Serial.print(millis() / 1000.);
    Serial.println();

    // send temperature
    Serial.print("!@THR#");
    #ifdef D_BMP280
    Serial.print(g_bmp280.readTemperature());
    Serial.print("#");
    #endif
    
    #ifdef D_MPL3115
    Serial.print(g_mpl3115.readTemp());
    Serial.print("#");
    #endif
    Serial.print(millis() / 1000.);
    Serial.println();

    #ifdef D_GPS
    // while data avaiable on RX...
    while (g_ss.available())
    {
        // read RX (GPS data)
        char l_ch = g_ss.read();

        // uncomment to see the GPS data flowing
        // Serial.write(l_ch);

        // did a new valid sentence come in ?
        if (g_gps.encode(l_ch))
            // set flag
            lv_new_data = true;

    } // end while

    if (lv_new_data)
    {
        g_gps.f_get_position(&lf_lat, &lf_lon, &lul_age);

        Serial.print("!@GPS#");
        Serial.print(lf_lat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : lf_lat, 6);
        Serial.print("#");
        Serial.print(lf_lon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : lf_lon, 6);
        Serial.print("#");
        Serial.print(g_gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : g_gps.satellites());
        Serial.print("#");
        Serial.print(g_gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : g_gps.hdop());
        Serial.print("#");
        Serial.print(millis() / 1000.);
        Serial.println();

    } // end if
    #endif

    #ifdef D_DEBUG
    Serial.print("Elapsed time: ");
    Serial.print(millis() - lul_ini);
    Serial.println(" ms.");
    #endif

    // D_TIM_WAIT - elapsed time
    lul_elp = D_TIM_WAIT - (millis() - lul_ini);

    // adiantado ?
    if (lul_elp > 0)
        // aguarda completar o tempo
        delay(lul_elp);

} // loop

// ------------------------------------------------------------------------------------------------
void setup_MPL3115() 
{
    // get sensor online
    g_mpl3115.begin();

    // configure the sensor

    // measure altitude above sea level in meters
    // g_mpl3115.setModeAltimeter();

    // measure pressure in Pascals from 20 to 110 kPa
    g_mpl3115.setModeBarometer();

    // set oversample to the recommended 128
    g_mpl3115.setOversampleRate(7);

    // enable all three pressure and temp event flags
    g_mpl3115.enableEventFlags();

} // setup_MPL3115

// ------------------------------------------------------------------------------------------------
void calibra() 
{
    #ifdef D_MPL3115
    // altitude calc
    float lf_Px;
    float lf_off_h;
    #endif

    // tempo inicial
    unsigned long lul_ini;
    // elapsed time
    unsigned long lul_elp;

    // for all calibration samples...
    for (int li_i = 0; li_i < D_CLB_SAMPLES; li_i++)
    {
        // get initial time (ms)
        lul_ini = millis();

        #ifdef D_BMP280
        // obtém a altitude 
        gf_alt_bmp += g_bmp280.readAltitude(g_QNH);
        #endif

        #ifdef D_MPL3115
        // measure altitude above sea level in meters
        lf_Px = 1. - pow(g_mpl3115.readPressure() / 101325, 0.1902632);
        lf_off_h = 60.;

        gf_alt_mpl += ((44330.77 * lf_Px) + lf_off_h);
        #endif

        #ifdef D_GPS
        #endif

        // D_TIM_WAIT - elapsed time
        lul_elp = D_TIM_WAIT - (millis() - lul_ini);

        // adiantado ?
        if (lul_elp > 0)
            // aguarda completar o tempo
            delay(lul_elp);

    } // end for

    #ifdef D_BMP280
    // calcula a média
    gf_alt_bmp /= D_CLB_SAMPLES;
    #endif

    #ifdef D_MPL3115
    // calcula a média
    gf_alt_mpl /= D_CLB_SAMPLES;
    #endif

} // calibra

// < the end >-------------------------------------------------------------------------------------
