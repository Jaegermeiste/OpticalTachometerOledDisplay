/*********************************************************************
RPM Tachometer with OLED digital and analog display
 *********************************************************************/
 
//One of the next two defines must be uncommented for the type of OLED display
        //SSD1306 is typically the 0.96" OLED
#define OLED_TYPE_SSD1306
        //SH1106 is typically a 1.3" OLED
//#define OLED_TYPE_SH1106

#ifdef OLED_TYPE_SH1106 
   #include <Adafruit_SH1106.h>
#endif

#ifdef OLED_TYPE_SSD1306
  #include <Adafruit_SSD1306.h>
#endif 

#include <Math.h>

namespace {
  const int OLED_RESET = 4;
  const int TEXT_SIZE_SMALL = 1;
  const int TEXT_SIZE_LARGE = 2;
  const int ONE_K = 1000;
  
  const int OLED_HEIGHT = 64;
  const int OLED_WIDTH = 128;
  const int YELLOW_SEGMENT_HEIGHT = 16;
  const int DISPLAY_FULL_BRIGHTNESS = 255;
  const int DISPLAY_DIM_BRIGHTNESS = 0;
  
  const int IR_LED_PIN_3 = 3;
  const int PHOTODIODE_PIN_2 = 2;
  const int INTERRUPT_ZERO_ON_PIN_2 = 0;
  
  const uint16_t DIAL_CENTER_X = OLED_WIDTH / 2;
  const uint16_t DIAL_RADIUS = (OLED_HEIGHT - YELLOW_SEGMENT_HEIGHT) - 1;
  const uint16_t DIAL_CENTER_Y = OLED_HEIGHT - 1;
  const uint16_t INDICATOR_LENGTH = DIAL_RADIUS - 5;
  const uint16_t INDICATOR_WIDTH = 5;
  const uint16_t LABEL_RADIUS = DIAL_RADIUS - 18;
  const int DIAL_LABEL_Y_OFFSET = 6;
  const int DIAL_LABEL_X_OFFSET = 4;
  
  long major_ticks[] = { 0, 2000, 4000, 6000 }; // Max is evenly divisible by 3
  const int MAJOR_TICK_COUNT = sizeof(major_ticks) / sizeof(major_ticks[0]);
  const int  MAJOR_TICK_LENGTH = 7;
  long minor_ticks[] = {1000, 3000, 5000};
  const int MINOR_TICK_COUNT = sizeof(minor_ticks) / sizeof(minor_ticks[0]);
  const int MINOR_TICK_LENGTH = 3;
  
  uint16_t dial_max_rpm = major_ticks[MAJOR_TICK_COUNT-1];
  const uint16_t MAX_RPM_GROWTH = 3000; // Ensure all increases are divisible by 3
  const int OBSERVED_MAX_RPM_TICK_LENGTH = -2;

  long observed_max_rpm = 0;
  
  const int HALF_CIRCLE_DEGREES = 180;
  const float PI_RADIANS = PI/HALF_CIRCLE_DEGREES;
  
  const double MILLIS_PER_SECOND = 1000.0;
  const double SECONDS_PER_MINUTE = 60.0;
  const long DISPLAY_TIMEOUT_INTERVAL = 120 * MILLIS_PER_SECOND;
  const long DISPLAY_DIM_INTERVAL = DISPLAY_TIMEOUT_INTERVAL/2;
  const long DISPLAY_UPDATE_INTERVAL = 250;
  const int  DISPLAY_AVERAGE_INTERVALS = 4;
  
  volatile unsigned long revolutions;
  
  unsigned long previous_revolutions = 0;
  unsigned long revolution_count[DISPLAY_AVERAGE_INTERVALS]; 
  unsigned long interval_millis[DISPLAY_AVERAGE_INTERVALS]; 
  unsigned int interval_index = 0;
  unsigned long previous_millis = 0;
  unsigned long last_sensor_time = 0;
  bool is_oled_display_on = false;
  bool is_oled_display_dim = false;

  const int EMA_PERIOD = 9;
  const int EMA_THRESHOLD = 1;  // Only consider changes of EMA_THRESHOLD as valid to avoid fractional noise
  double denom_ema = 0.0;
  double alpha = 0.0;
  volatile double previous_ema_rpm = 0.0;
}

#ifdef OLED_TYPE_SH1106
   Adafruit_SH1106 display(OLED_RESET);
#else
   Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT);
#endif

void setup() {
  Serial.begin(9600);

  // https://github.com/impexeris/OpticalTachometerOledDisplay/blob/master/OpticalTachometerOledDisplay.ino
  delay (MILLIS_PER_SECOND/3); //initial delay to avoid display flickering due to power-on noises
  
  initOledDisplayWithI2CAddress(0x3C);
  display.setTextColor(WHITE);
  initArrays();
  emaSetup();
  
  turnOnIrLED();
  attachPhotodiodeToInterrruptZero();
  last_sensor_time = millis();
  turnOnDisplay();
}

void initArrays() {
  memset(revolution_count,0,sizeof(revolution_count));
  memset(interval_millis,0,sizeof(interval_millis));
}

void recalculateTicks(int max_rpm) {
  // Increase the max
  while (max_rpm > dial_max_rpm) {
    dial_max_rpm += MAX_RPM_GROWTH;
  }

  // Calculate dial spacing
  int dial_major_interval = dial_max_rpm / (MAJOR_TICK_COUNT - 1);

  // Update major ticks
  for (int i = 0; i < MAJOR_TICK_COUNT; i++)
  {
    if (i == 0) {
      // First tick
      major_ticks[i] = 0;
    }
    else if (i == (MAJOR_TICK_COUNT - 1)) {
      // Last tick
      major_ticks[i] = dial_max_rpm;
    }
    else {
      // Middle ticks
      major_ticks[i] = floor(dial_major_interval * i);  
    }
  }

  // Update minor ticks
  for (int i = 0; i < MINOR_TICK_COUNT; i++)
  {
    minor_ticks[i] = floor((dial_major_interval / 2) * (i + 1));  
  }
}

void loop() {
  unsigned long current_millis = millis();
  if (current_millis - last_sensor_time >= DISPLAY_TIMEOUT_INTERVAL) {
    turnOffDisplay();
  } else if (current_millis - last_sensor_time >= DISPLAY_DIM_INTERVAL) {
    dimDisplay();
  }  
  
  if (current_millis - previous_millis >= DISPLAY_UPDATE_INTERVAL) {
    previous_millis = current_millis;

    long rpm = calculateRpm();
    long ema_rpm = emaRpm(rpm);
  
    // Update the max tick as appropriate
    if (ema_rpm > observed_max_rpm) {
      observed_max_rpm = ema_rpm;
    }
  
    // Scale up if we go off the chart
    if (ema_rpm > dial_max_rpm) {
      recalculateTicks(ema_rpm);
    }
    
    updateDisplay(ema_rpm);
  }
}

void initOledDisplayWithI2CAddress(uint8_t i2c_address) {
  #ifdef OLED_TYPE_SH1106
    display.begin(SH1106_SWITCHCAPVCC, i2c_address);
  #else
    display.begin(SSD1306_SWITCHCAPVCC, i2c_address);
  #endif
}

void turnOnDisplay() {
  commandOledOn();
}

void commandOledOn() {
  #ifdef OLED_TYPE_SH1106
    display.SH1106_command(SH1106_DISPLAYON);
  #else
    display.ssd1306_command(SSD1306_DISPLAYON); 
  #endif
  is_oled_display_on = true;
  oledDisplayFullBrightness();
}

void turnOffDisplay() {
  #ifdef OLED_TYPE_SH1106
    display.SH1106_command(SH1106_DISPLAYOFF);
  #else
    display.ssd1306_command(SSD1306_DISPLAYOFF); 
  #endif
  is_oled_display_on = false;
  is_oled_display_dim = false;
}

void dimDisplay() {
  oledDisplayDim();
}

void oledDisplayDim() {
  #ifdef OLED_TYPE_SSD1306 
    display.dim(true); 
  #endif
  is_oled_display_dim = true;
}

void oledDisplayFullBrightness() {
  #ifdef OLED_TYPE_SSD1306 
    display.dim(false); 
  #endif
  is_oled_display_dim = false;
}

void turnOnIrLED() {
  pinMode(IR_LED_PIN_3, OUTPUT);
  digitalWrite(IR_LED_PIN_3, HIGH);
}

void attachPhotodiodeToInterrruptZero() {
  pinMode(PHOTODIODE_PIN_2, INPUT_PULLUP);
  attachInterrupt(INTERRUPT_ZERO_ON_PIN_2, incrementRevolution, FALLING);
}

void incrementRevolution() {
  revolutions++;
}

void updateDisplay(long rpm) {
  if (rpm > 0) {
    last_sensor_time = millis();
    if (!is_oled_display_on || is_oled_display_dim) {
      turnOnDisplay();
    }
  }
  if (is_oled_display_on) {
    display.clearDisplay();
    drawRpmBanner(rpm);
    drawDial(rpm);
    display.display();
  }
}

void emaSetup(){
  if (denom_ema == 0){
    if (!(EMA_PERIOD == -1)){
      denom_ema = 1 + EMA_PERIOD;
    }
    else {
      denom_ema = 1;
    }
  }

  if (previous_ema_rpm <= 0) {
    previous_ema_rpm = 0.0;
  }

  alpha = 2.0 / denom_ema;
}

// Avoid hysteresis
long emaRpm(long raw_rpm) {
  // Calculate EMA
  double ema_rpm = (long) (alpha * raw_rpm + (1 - alpha) * previous_ema_rpm);

  // Can't have an RPM < 0
  if (ema_rpm < 0) {
    ema_rpm = 0.0;
  }

  // Ignore sub threshold RPM changes (avoid noise)
  if (abs((long)previous_ema_rpm - (long)ema_rpm) < EMA_THRESHOLD) {
    ema_rpm = previous_ema_rpm;
  }

  // Update previous
  previous_ema_rpm = ema_rpm;

  return (long)ema_rpm;
}

long calculateRpm() {
  unsigned long current_millis = millis();
  unsigned long current_revolutions = revolutions;
  unsigned long previous_display_millis;
  unsigned long previous_revolutions;
    
  queueIntervalRevolution(current_revolutions, current_millis);
  previous_display_millis = getIntervalMillis();
  previous_revolutions = getIntervalRevolutions();

  unsigned long elapsed_millis =  current_millis - previous_display_millis;
  float elapsed_seconds = ((elapsed_millis * 1.0) / MILLIS_PER_SECOND);
  float delta_revolutions = (current_revolutions - previous_revolutions) * 1.0;

  long rpm = (long) ((delta_revolutions / elapsed_seconds) * SECONDS_PER_MINUTE);
  return rpm;
}

void queueIntervalRevolution(unsigned long revolution_value, unsigned long milliseconds) {
  interval_index++;
  int queue_index = (int)(interval_index % DISPLAY_AVERAGE_INTERVALS);
  revolution_count[queue_index] = revolution_value; 
  interval_millis[queue_index] = milliseconds;
}

unsigned long getIntervalMillis() {
  int index_front_of_queue = (int)((interval_index + 1)  % DISPLAY_AVERAGE_INTERVALS);
  return interval_millis[index_front_of_queue];
}

unsigned long getIntervalRevolutions() {
  int index_front_of_queue = (int)((interval_index + 1)  % DISPLAY_AVERAGE_INTERVALS);
  return revolution_count[index_front_of_queue];
}

void drawRpmBanner(long rpm_value) {
  display.setCursor(0, 0);

  display.setTextSize(TEXT_SIZE_LARGE);
  display.print("RPM: ");
  display.print((long)rpm_value);
}

void drawDial(long rpm_value) {
  display.drawCircle(DIAL_CENTER_X, DIAL_CENTER_Y, DIAL_RADIUS, WHITE);
  drawTickMarks();
  drawMajorTickLabels();
  drawIndicatorHand(rpm_value);
}

void drawTickMarks() {
  drawTicks(major_ticks, MAJOR_TICK_COUNT, MAJOR_TICK_LENGTH);
  drawTicks(minor_ticks, MINOR_TICK_COUNT, MINOR_TICK_LENGTH);

  // Draw max observed tick
  drawTick(observed_max_rpm, OBSERVED_MAX_RPM_TICK_LENGTH);
}

void drawTicks(const long ticks[], int tick_count, int tick_length) {
  for (int tick_index = 0; tick_index < tick_count; tick_index++) {
    drawTick(ticks[tick_index], tick_length);
  }
}

void drawTick(const long rpm_tick_value, int tick_length) {
  float tick_angle = (HALF_CIRCLE_DEGREES * getPercentMaxRpm(rpm_tick_value)) + HALF_CIRCLE_DEGREES;
  uint16_t dial_x = getCircleXWithLengthAndAngle(DIAL_RADIUS - 1, tick_angle);
  uint16_t dial_y = getCircleYWithLengthAndAngle(DIAL_RADIUS - 1, tick_angle);
  uint16_t tick_x = getCircleXWithLengthAndAngle(DIAL_RADIUS - tick_length, tick_angle);
  uint16_t tick_y = getCircleYWithLengthAndAngle(DIAL_RADIUS - tick_length, tick_angle);
  display.drawLine(dial_x, dial_y, tick_x, tick_y, WHITE);
}

float getPercentMaxRpm(long value) {
  float ret_value = (value * 1.0)/(dial_max_rpm * 1.0);
  return ret_value;
}

float getCircleXWithLengthAndAngle(uint16_t radius, float angle) {
  return DIAL_CENTER_X + radius * cos(angle*PI_RADIANS);
};

float getCircleYWithLengthAndAngle(uint16_t radius, float angle) {
  return DIAL_CENTER_Y + radius * sin(angle*PI_RADIANS);
};

void drawMajorTickLabels() {
  display.setTextSize(TEXT_SIZE_SMALL);
  for (int label_index = 0; label_index < MAJOR_TICK_COUNT; label_index++) {
    long rpm_tick_value = major_ticks[label_index];
    float tick_angle = (HALF_CIRCLE_DEGREES * getPercentMaxRpm(rpm_tick_value)) + HALF_CIRCLE_DEGREES;
    uint16_t dial_x = getCircleXWithLengthAndAngle(LABEL_RADIUS, tick_angle);
    uint16_t dial_y = getCircleYWithLengthAndAngle(LABEL_RADIUS, tick_angle);
    display.setCursor(dial_x - DIAL_LABEL_X_OFFSET, dial_y - DIAL_LABEL_Y_OFFSET);
    int label_value = rpm_tick_value / ONE_K;
    display.print(label_value);
  }
}

void drawIndicatorHand(long rpm_value) {
    float indicator_angle = (HALF_CIRCLE_DEGREES * getPercentMaxRpm(rpm_value)) + HALF_CIRCLE_DEGREES;
    uint16_t indicator_top_x = getCircleXWithLengthAndAngle(INDICATOR_LENGTH, indicator_angle);
    uint16_t indicator_top_y = getCircleYWithLengthAndAngle(INDICATOR_LENGTH, indicator_angle);

  display.drawTriangle(DIAL_CENTER_X - INDICATOR_WIDTH / 2,
                       DIAL_CENTER_Y,DIAL_CENTER_X + INDICATOR_WIDTH / 2,
                       DIAL_CENTER_Y,
                       indicator_top_x, 
                       indicator_top_y, 
                       WHITE);
}
