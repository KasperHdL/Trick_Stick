#include "Wire.h"


//debug
bool debug = true;
bool debug_photo = false;
bool debug_imu = false;


//score
unsigned long score = 0;

float sum_multiplier = 1;
int score_base_increment = 1;

//score multiplier
float score_wild_balancing = score_base_increment * 10;

//states
bool is_balancing = false;

int timerEnd = 0;

//thresholds
float acc_epsilon = 0.5f;


///////////////

//Alcohol Meter
int pin_alcohol = 0;
int alcohol_value;
bool alcohol_calibrated = false;

int alcohol_samples = 10;
int alcohol_time_between_samples = 500;

int alcohol_index = 0;
int alcohol_sum;

float alcohol_multiplier = 1;

float sensor_volt = 0;
float RS_air = 0;

//Photo cell
bool photo_calibrated = false;
int photo_value;
int pin_photo = 2;

int photo_samples = 30;
int photo_time_between_samples = 200;

int covered_sum = 0;
int covered_index = 0;
int covered_average;

int uncovered_sum = 0;
int uncovered_index = 0;
int uncovered_average;

//IMU - Acceleration, Gyro and Compass data
float Axyz[3];
float Gxyz[3];
float Mxyz[3];


// LCD

// include the library code:
#include <LiquidCrystal.h>

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);



void setup() {
  Wire.begin();
  Serial.begin(38400);
  
  pinMode(15,OUTPUT);   // set the heaterSelPin as digital output.
  digitalWrite(15,LOW); // Start to heat the sensor  

  lcd.begin(16, 2);
  // set the cursor to (0,0):

  setup_imu(0);
}

void loop() {

  if (!photo_calibrated || !alcohol_calibrated) {
    if(!alcohol_calibrated){
      alcohol_calibration();
      return;
    }
    if(!photo_calibrated)
      photo_calibration();
    return;
  }

  loop_imu();
  loop_photo();

  loop_game();
  display_score();
}

void loop_game() {
  
  float acc_magnitude_sq = -1;
  if (is_balancing) {
    acc_magnitude_sq = abs((Axyz[0] * Axyz[0] + Axyz[1] * Axyz[1]));

    

    if (acc_magnitude_sq > acc_epsilon * acc_epsilon) {
      sum_multiplier = score_wild_balancing;
    } else {
      sum_multiplier = 1;
    }

    sum_multiplier *= alcohol_multiplier;

  }

  if (debug_imu) {
    Serial.print("\tacc mag: \t");
    Serial.print(acc_magnitude_sq);
    Serial.print("\tscore: \t");
    Serial.print(score);
    Serial.print(" -\t");
    Serial.print(sum_multiplier);
    Serial.print("x");
  }

  if(debug_imu || debug_photo)
    Serial.println();
  if (is_balancing)
    score += score_base_increment * sum_multiplier;
}


void display_score() {
  lcd.setCursor(0, 0);
  lcd.print(String(is_balancing ? "Balancing    " : "failing      "));
  lcd.setCursor(0, 1);
  lcd.print(String(score) + " - " + String(sum_multiplier) + "x       ");
}

/////////////////////
///// Alcohol Sensor


void alcohol_calibration() {
  alcohol_value = analogRead(pin_alcohol);

  Serial.print(alcohol_value);
  Serial.print(" - ");

  if (alcohol_index < alcohol_samples) {
    if (alcohol_index == 0) {
      Serial.println("calibrating Alcohol");
      timerEnd = millis() + (alcohol_time_between_samples * alcohol_samples);
      
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Calibrating");
      lcd.setCursor(0, 1);
      lcd.print("Clean Air");
    }
    alcohol_sum += alcohol_value;
    
    lcd.setCursor(13,1);
    lcd.print(String((timerEnd - millis())/1000) + " s");
    delay(alcohol_time_between_samples);
  } else if(alcohol_index < alcohol_samples * 2){
    if(alcohol_index == alcohol_samples){
        
   
      sensor_volt = ((float)alcohol_sum / alcohol_samples)/1024*5.0;
      RS_air = sensor_volt/(5.0-sensor_volt); // omit *R16

      alcohol_sum = 1023;
      Serial.println();
      Serial.println("begin exhaling for as long as possible in 3 second");
      
      lcd.clear();
      lcd.setCursor(0, 0);

      lcd.print("Calibrating");
      lcd.setCursor(0, 1);
      lcd.print("Alc. Level in 6s");
      delay(1000);
      
      lcd.setCursor(0, 1);
      lcd.print("Alc. Level in 5s");
      delay(1000);
      lcd.setCursor(0, 1);
      lcd.print("Alc. Level in 4s");
      delay(1000);
      
      lcd.clear();
      lcd.setCursor(0, 0);

      lcd.print("Prepare to");
      lcd.setCursor(0, 1);
      lcd.print("Exhale in 3s");
      delay(1000);
      lcd.setCursor(0, 1);
      lcd.print("Exhale in 2s");
      delay(1000);
      lcd.setCursor(0, 1);
      lcd.print("Exhale in 1s");
      delay(1000);
      
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("EXHALE!");
      lcd.setCursor(0, 1);
      lcd.print("KEEP GOING");
      
      timerEnd = millis() + (alcohol_time_between_samples * alcohol_samples);
    }

      
    if(alcohol_value < alcohol_sum) alcohol_sum = alcohol_value;

    
    lcd.setCursor(13,1);
    lcd.print(String((timerEnd - millis())/1000) + " s");
    Serial.println((timerEnd - millis())/1000);
    delay(alcohol_time_between_samples);
  } else {
    alcohol_calibrated = true;

    
    sensor_volt=(float)alcohol_value/1024*5.0;
    float RS_gas = sensor_volt/(5.0-sensor_volt); // omit *R16
   
    /*-Replace the name "R0" with the value of R0 in the demo of First Test -*/
    float ratio = RS_air/RS_gas;  // ratio = RS/R0 
    
    alcohol_multiplier = (ratio / 5) + 1; 
    Serial.println();
    Serial.print("Alcohol calibrated multiplier: ");
    Serial.println(alcohol_multiplier);
  
    lcd.clear();
    lcd.setCursor(0, 0);

    lcd.print("Drunkness");
    lcd.setCursor(0, 1);
    lcd.print("Multiplier " + String(alcohol_multiplier) + "x");

    delay(2000);

  }
  alcohol_index++;
}



/////////////////////
///// Photo cell

void photo_calibration() {
  photo_value = analogRead(pin_photo);

  Serial.print(photo_value);
  Serial.print(" - ");

  if (covered_index++ < photo_samples) {
    if (covered_index == 1) {
      Serial.println("calibrating");
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Calibrating");
      lcd.setCursor(0, 1);
      lcd.print("Covered");
    }
    covered_sum += photo_value;
    lcd.setCursor(12,1);
    lcd.print(String(covered_sum/covered_index));
  } else if (uncovered_index++ < photo_samples) {
    if (uncovered_index == 1) {

      covered_average = covered_sum / photo_samples;
      Serial.println();
      Serial.print("covered_average: ");
      Serial.print(covered_average);

      lcd.clear();
      lcd.setCursor(0, 0);

      lcd.print("Calibrating");
      lcd.setCursor(0, 1);
      lcd.print("Uncovered");
      Serial.println("begin uncovered calibration in 2s");
      delay(2000);
    }
    uncovered_sum += photo_value;
    
    lcd.setCursor(12,1);
    lcd.print(String(uncovered_sum/uncovered_index));
  } else {
    photo_calibrated = true;

    uncovered_average = uncovered_sum / photo_samples;

    Serial.print("uncovered_average: ");
    Serial.print(uncovered_average);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Game Starting!");

    delay(500);

    lcd.clear();

  }


  delay(photo_time_between_samples);
}

void loop_photo() {

  photo_value = analogRead(pin_photo);


  int cov_diff = abs(photo_value - covered_average);
  int uncov_diff = abs(photo_value - uncovered_average);


  if (debug_photo) {

    Serial.print(" value: ");
    Serial.print(photo_value);

    Serial.print("\t diffs: ");
    Serial.print(cov_diff);
    Serial.print("/");
    Serial.print(uncov_diff);

    is_balancing = cov_diff < uncov_diff;
    Serial.print("\t diff_balancing: ");
    Serial.print(is_balancing);

  }

  is_balancing = cov_diff < uncov_diff;



}




//////////////////////
////// IMU

// I2Cdev and MPU9250 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU9250.h"
#include "BMP180.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU9250 accelgyro;
I2Cdev   I2C_M;

uint8_t buffer_m[6];


int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t   mx, my, mz;



float heading;
float tiltheading;



#define sample_num_mdate  5000

volatile float mx_sample[3];
volatile float my_sample[3];
volatile float mz_sample[3];

static float mx_centre = 0;
static float my_centre = 0;
static float mz_centre = 0;

volatile int mx_max = 0;
volatile int my_max = 0;
volatile int mz_max = 0;

volatile int mx_min = 0;
volatile int my_min = 0;
volatile int mz_min = 0;

float temperature;
float pressure;
float atm;
float altitude;
BMP180 Barometer;

int imu_delay = 0;

void setup_imu(int loop_delay) {
  imu_delay = loop_delay;

  // initialize device
  Serial.println("Initializing I2C devices...");
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Initing IMU");
  accelgyro.initialize();
  Barometer.init();

  // verify connection
  Serial.println("Testing device connections...");
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("testing IMU");
  lcd.setCursor(0, 1);
  lcd.print("connections");
  Serial.println(accelgyro.testConnection() ? "MPU9250 connection successful" : "MPU9250 connection failed");
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("MPU connection");
  lcd.setCursor(0, 1);
  lcd.print(accelgyro.testConnection() ? "successful" : "failed");

  delay(1000);
  Serial.println("     ");

  //  Mxyz_init_calibrated ();
}

void loop_imu() {
  loop_6dof();

  delay(imu_delay);
}

void loop_6dof() {
  getAccel_Data();
  getGyro_Data();
  getCompassDate_calibrated(); // compass data has been calibrated here
  getHeading();               //before we use this function we should run 'getCompassDate_calibrated()' frist, so that we can get calibrated data ,then we can get correct angle .
  getTiltHeading();

  if (debug_imu) {
    Serial.println("calibration parameter: ");
    Serial.print(mx_centre);
    Serial.print("         ");
    Serial.print(my_centre);
    Serial.print("         ");
    Serial.println(mz_centre);
    Serial.println("     ");


    Serial.println("Acceleration(g) of X,Y,Z:");
    Serial.print(Axyz[0]);
    Serial.print(",");
    Serial.print(Axyz[1]);
    Serial.print(",");
    Serial.println(Axyz[2]);
    Serial.println("Gyro(degress/s) of X,Y,Z:");
    Serial.print(Gxyz[0]);
    Serial.print(",");
    Serial.print(Gxyz[1]);
    Serial.print(",");
    Serial.println(Gxyz[2]);
    Serial.println("Compass Value of X,Y,Z:");
    Serial.print(Mxyz[0]);
    Serial.print(",");
    Serial.print(Mxyz[1]);
    Serial.print(",");
    Serial.println(Mxyz[2]);
    Serial.println("The clockwise angle between the magnetic north and X-Axis:");
    Serial.print(heading);
    Serial.println(" ");
    Serial.println("The clockwise angle between the magnetic north and the projection of the positive X-Axis in the horizontal plane:");
    Serial.println(tiltheading);
    Serial.println("   ");
  }

}

void loop_3dof() {

  temperature = Barometer.bmp180GetTemperature(Barometer.bmp180ReadUT()); //Get the temperature, bmp180ReadUT MUST be called first
  pressure = Barometer.bmp180GetPressure(Barometer.bmp180ReadUP());//Get the temperature
  altitude = Barometer.calcAltitude(pressure); //Uncompensated caculation - in Meters
  atm = pressure / 101325;

  if (debug_imu) {
    Serial.print("Temperature: ");
    Serial.print(temperature, 2); //display 2 decimal places
    Serial.println("deg C");

    Serial.print("Pressure: ");
    Serial.print(pressure, 0); //whole number only.
    Serial.println(" Pa");

    Serial.print("Ralated Atmosphere: ");
    Serial.println(atm, 4); //display 4 decimal places

    Serial.print("Altitude: ");
    Serial.print(altitude, 2); //display 2 decimal places
    Serial.println(" m");

    Serial.println();
  }
}



void getHeading(void)
{
  heading = 180 * atan2(Mxyz[1], Mxyz[0]) / PI;
  if (heading < 0) heading += 360;
}

void getTiltHeading(void)
{
  float pitch = asin(-Axyz[0]);
  float roll = asin(Axyz[1] / cos(pitch));

  float xh = Mxyz[0] * cos(pitch) + Mxyz[2] * sin(pitch);
  float yh = Mxyz[0] * sin(roll) * sin(pitch) + Mxyz[1] * cos(roll) - Mxyz[2] * sin(roll) * cos(pitch);
  float zh = -Mxyz[0] * cos(roll) * sin(pitch) + Mxyz[1] * sin(roll) + Mxyz[2] * cos(roll) * cos(pitch);
  tiltheading = 180 * atan2(yh, xh) / PI;
  if (yh < 0)    tiltheading += 360;
}



void Mxyz_init_calibrated ()
{

  Serial.println(F("Before using 9DOF,we need to calibrate the compass frist,It will takes about 2 minutes."));
  Serial.print("  ");
  Serial.println(F("During  calibratting ,you should rotate and turn the 9DOF all the time within 2 minutes."));
  Serial.print("  ");
  Serial.println(F("If you are ready ,please sent a command data 'ready' to start sample and calibrate."));

  while (!Serial.find("ready"));

  Serial.println("  ");
  Serial.println("ready");
  Serial.println("Sample starting......");
  Serial.println("waiting ......");

  get_calibration_Data ();

  Serial.println("     ");
  Serial.println("compass calibration parameter ");
  Serial.print(mx_centre);
  Serial.print("     ");
  Serial.print(my_centre);
  Serial.print("     ");
  Serial.println(mz_centre);
  Serial.println("    ");
}


void get_calibration_Data ()
{
  for (int i = 0; i < sample_num_mdate; i++)
  {
    get_one_sample_date_mxyz();
    /*
      Serial.print(mx_sample[2]);
      Serial.print(" ");
      Serial.print(my_sample[2]);                            //you can see the sample data here .
      Serial.print(" ");
      Serial.println(mz_sample[2]);
    */



    if (mx_sample[2] >= mx_sample[1])mx_sample[1] = mx_sample[2];
    if (my_sample[2] >= my_sample[1])my_sample[1] = my_sample[2]; //find max value
    if (mz_sample[2] >= mz_sample[1])mz_sample[1] = mz_sample[2];

    if (mx_sample[2] <= mx_sample[0])mx_sample[0] = mx_sample[2];
    if (my_sample[2] <= my_sample[0])my_sample[0] = my_sample[2]; //find min value
    if (mz_sample[2] <= mz_sample[0])mz_sample[0] = mz_sample[2];

  }

  mx_max = mx_sample[1];
  my_max = my_sample[1];
  mz_max = mz_sample[1];

  mx_min = mx_sample[0];
  my_min = my_sample[0];
  mz_min = mz_sample[0];



  mx_centre = (mx_max + mx_min) / 2;
  my_centre = (my_max + my_min) / 2;
  mz_centre = (mz_max + mz_min) / 2;

}






void get_one_sample_date_mxyz()
{
  getCompass_Data();
  mx_sample[2] = Mxyz[0];
  my_sample[2] = Mxyz[1];
  mz_sample[2] = Mxyz[2];
}


void getAccel_Data(void)
{
  accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  Axyz[0] = (double) ax / 16384;
  Axyz[1] = (double) ay / 16384;
  Axyz[2] = (double) az / 16384;
}

void getGyro_Data(void)
{
  accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  Gxyz[0] = (double) gx * 250 / 32768;
  Gxyz[1] = (double) gy * 250 / 32768;
  Gxyz[2] = (double) gz * 250 / 32768;
}

void getCompass_Data(void)
{
  I2C_M.writeByte(MPU9150_RA_MAG_ADDRESS, 0x0A, 0x01); //enable the magnetometer
  delay(10);
  I2C_M.readBytes(MPU9150_RA_MAG_ADDRESS, MPU9150_RA_MAG_XOUT_L, 6, buffer_m);

  mx = ((int16_t)(buffer_m[1]) << 8) | buffer_m[0] ;
  my = ((int16_t)(buffer_m[3]) << 8) | buffer_m[2] ;
  mz = ((int16_t)(buffer_m[5]) << 8) | buffer_m[4] ;

  Mxyz[0] = (double) mx * 1200 / 4096;
  Mxyz[1] = (double) my * 1200 / 4096;
  Mxyz[2] = (double) mz * 1200 / 4096;
}

void getCompassDate_calibrated ()
{
  getCompass_Data();
  Mxyz[0] = Mxyz[0] - mx_centre;
  Mxyz[1] = Mxyz[1] - my_centre;
  Mxyz[2] = Mxyz[2] - mz_centre;
}

