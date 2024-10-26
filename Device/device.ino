#include "AZ3166WiFi.h"
#include "OledDisplay.h"
#include "EEPROMInterface.h"
#include "SerialLog.h"
#include "DevKitMQTTClient.h"
#include <Sensor.h>


// temp and humidity sensors
DevI2C *i2c;
HTS221Sensor *sensor;
float humidity = 0;
float temperature = 0;

// magentometer sensor
LIS2MDLSensor *magnetometer;
int axes[3];
float declination_offset_radians = 0;
double M_PIE = 3.14159265358979323846;
int DEC_EW = 5;
int deg = 0;

// wifi code - only used when Wifiinit() is fired, used for when hard-coding wifi credentials
char wifiBuff[128];
static bool isConnected;
char ssid[] = "";    
char password[] = "";  
int status = WL_IDLE_STATUS; 


void setup() {
    // initialise display
    Screen.init();

    // initialise the sensors
    i2c = new DevI2C(D14, D15);
    sensor = new HTS221Sensor(*i2c);
    sensor -> init(NULL);
    magnetometer = new LIS2MDLSensor(*i2c);
    magnetometer->init(NULL);
    // set magnetic deviation
     SetDeclination(DEC_EW,'W');

    // init wifi connection
    // InitWiFi(); // this is only used if we want to hard code the wifi settings

    Screen.clean();
    // wipe current IoT Hub connection string in EEPROM
    clearConnectionString();
    WiFi.begin();

    if(WiFi.begin() == WL_CONNECTED)
  {
    IPAddress ip = WiFi.localIP();
    snprintf(wifiBuff, 128, "WiFi Connected\r\n%s\r\n%s\r\n \r\n",WiFi.SSID(),ip.get_address());
    Screen.print(1, "Connecting");
    Screen.print(2, wifiBuff);
    isConnected = true;
    Screen.print(3, ip.get_address());
    delay(3000);

   // Write the connection string to EEPROM as an array of uint8_t
    EEPROMInterface eeprom;
    const char connString[] = "HostName=trafficsensor.azure-devices.net;DeviceId=temphumditysensor;SharedAccessKey=6viw46V75Ssq4N7LmogIH3y0pjix7tf64QvX1Fi8Iv8=";
    eeprom.write((uint8_t*)connString, strlen(connString), AZ_IOT_HUB_ZONE_IDX);   
    
    // connect to IoT Hub, LED will turn blue on board with successful connection
    DevKitMQTTClient_Init(true);

  }
    
}

// used to wipe the IoT Hub string from EEPROM
void clearConnectionString() {
    EEPROMInterface eeprom;

    uint8_t *cleanBuff = (uint8_t*) malloc(AZ_IOT_HUB_MAX_LEN);
    memset(cleanBuff, 0x00, AZ_IOT_HUB_MAX_LEN);
    eeprom.write(cleanBuff, AZ_IOT_HUB_MAX_LEN, AZ_IOT_HUB_ZONE_IDX);
    free(cleanBuff);
}

 
void sendData(const char *data) {
  time_t t = time(NULL);
  char buf[sizeof "2011-10-08T07:07:09Z"];
  strftime(buf, sizeof buf, "%FT%TZ", gmtime(&t));
 
  EVENT_INSTANCE* message = DevKitMQTTClient_Event_Generate(data, MESSAGE);

  
 
  DevKitMQTTClient_Event_AddProp(message, "$$CreationTimeUtc", buf);
  DevKitMQTTClient_Event_AddProp(message, "$$MessageSchema", "temperature;v1");
  DevKitMQTTClient_Event_AddProp(message, "$$ContentType", "JSON");
 
  DevKitMQTTClient_SendEventInstance(message);
}

void loop() {
 
        Screen.clean();
     
        // enable sensor and get value
        sensor -> enable();
        sensor -> getTemperature(&temperature);
        sensor -> getHumidity(&humidity);

         // read magnetometer
         magnetometer->getMAxes(axes);
          // calculate heading 
        float heading = Getheading(axes[0],axes[1]);
        heading += declination_offset_radians;
        
        // Correct for when signs are reversed.
        if(heading < 0)
            heading += 2*M_PIE;
        
        // Check for wrap due to addition of declination.
        if(heading > 2* M_PIE)
            heading -= 2* M_PIE;

        heading = heading * 180/M_PIE;
        deg = (int)heading;

        // display direction
        char buff0[128];
        sprintf(buff0,  "Magnetometer: \r\n x:%d   y:%d    z:%d \r\n heading : %d  ", axes[0], axes[1], axes[2], deg);
        Screen.print(buff0);
        delay(3000);

        
        // display temperature to screen
        Screen.clean();
        char buff[16];
        sprintf(buff, "Temp and Humid: \r\n temp:%s \r\n hum:%s", f2s(temperature, 1), f2s(humidity, 1));
        Screen.print(buff);
        delay(1500);
      

    // send temp data to iot Hub
    char sensorData[200];

       sprintf_s(sensorData,
          sizeof(sensorData),
       // "{\"temperature\":%s, \"humidity\":%s}", f2s(temperature, 1), f2s(humidity, 1));
       "{\"temperature\":%s, \"humidity\":%s, \"magnetometer\":%s}", f2s(temperature, 1), f2s(humidity, 1), f2s(deg, 1));
 
   sendData(sensorData);
 
    // should always normally be a delay when looping!
    delay(30000);
     
}
 
// outputs current IP
void printCurrentNet() {
   
    // print your WiFi IP address to display:  
    IPAddress ip = WiFi.localIP();
    Screen.print(3, ip.get_address());
 
}

// used for magnetometer
float Getheading(double x, double y) 
{
  float head = atan2(y, x); // Slope Y, Slope X
  return head;
}

void SetDeclination(int declination_degs , char declination_dir )
{
    // Convert declination to decimal degrees
    switch(declination_dir)
    {
      // North and East are positive   
      case 'E': 
        declination_offset_radians = declination_degs  * (M_PIE / 180);
        break;
        
      // South and West are negative    
      case 'W':
        declination_offset_radians =  0 - ( declination_degs  * (M_PIE/ 180) ) ;
        break;
    } 
}

// used only when hard-coded wifi credentials will be used
void InitWiFi()
{
  if(WiFi.begin() != WL_CONNECTED)
  {
    snprintf(wifiBuff, 128, "No WiFi\r\nEnter AP Mode\r\nto config\r\n                 \r\n");
    Screen.print(wifiBuff);
   Screen.print("Connecting to:");
    Screen.print(1, ssid);
     // Connect to WPA/WPA2 network:
   status = WiFi.begin(ssid, password);
    // Wait 10 seconds for connection:
    delay(10000);
  }
}

