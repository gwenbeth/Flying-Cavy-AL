// F5L altitude limiter by Gwen Scogin - 2026

#include <Servo.h> 

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP5xx.h"

#include <Adafruit_NeoPixel.h>

// How many internal neopixels do we have? some boards have more than one!
#define NUMPIXELS        1

Adafruit_NeoPixel pixels(NUMPIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);


#define SEALEVELPRESSURE_HPA (1013.25)

#define DEBUG_OUTPUT 1

Adafruit_BMP5xx bmp; // Create BMP5xx object

bmp5xx_powermode_t desiredMode = BMP5XX_POWERMODE_NORMAL; // Cache desired power mode

Servo t_out;

#define DELAY_TIME 20  // this is the 20ms between servo pulses from the receiver
#define ALT_HISTORY_SIZE 25
#define OVERSHOOT_FACTOR 1.2
#define THROTTLE_LOW_US 1150       // throttle must be below this...
                                   // 1150 because my rx is goes from
                                   // 1100-1900 even though the tx
                                   // says otherwise
#define THROTTLE_LOW_MS 1000       // ...for this long after power up before arming is allowed
#define THROTTLE_MIN_VALID_US 800  // pulseIn returns 0 on timeout; don't treat no signal as low
#define SENSOR_FAIL_THRESHOLD 3    // consecutive failed reads before we latch a sensor fault


// colors in the Okabe Ito pallet for friendlyness for color deficient vision
#define BLACK (0x000000)
#define ORANGE (0xE69F00)
#define CYAN (0x56B4E9)
#define GREEN (0x009E73)
#define YELLOW (0xF0E442)
#define BLUE (0x0072B2)
#define RED (0xD55E00)
#define PURPLE (0xCC79A7)

#define WAIT_COLOR  BLUE     // waiting for throttle low after power up
#define READY_COLOR GREEN
#define ARMED_COLOR CYAN
#define DONE_COLOR PURPLE
#define FOUL_COLOR RED
#define FAULT_COLOR ORANGE   // sensor hardware failure discovered after setup()


#define INPUT_PIN 2
#define OUTPUT_PIN 4
#define BUTTON_PIN 11

enum State_t {
  Start,
  Wait_For_Valid_Throttle,
  Can_Arm,
  Armmed,
  Done_Cant_Rearm,
  Done_Can_Rearm,
  Fault
};

State_t state = Start;

enum Reason_t {
  Ready,
  Altitude,
  Time,
  Foul
};

Reason_t reason = Ready;


int color = WAIT_COLOR;
uint32_t throttle_low_start = 0;
float altitude_list[] = {80.0,100.0,120.0};
int timer_list[] = {15000,30000,30000};

unsigned long timer = timer_list[0];
float altitude= altitude_list[0];

float base_altitude=0;
float cutoff_altitude = 0;
float start_altitude = 0;

unsigned long base_timer = 0;
unsigned long loop_counter = 0;
int arm_count = 0;
int sensor_fail_count = 0;

// We are maintaining a history of the past readings of altitude.  When 
// we compute the vertical speed (vspd) we will look further back in time
// than just the previous reading. This is because loop runs fast enough
// that there might not be a detectable change in altitude.  
float previous_altitude_arr[ALT_HISTORY_SIZE];

int pa_idx = 0; 

// there is no reason to do this every time through the loop
float vspd_correction = 1000.0 / (DELAY_TIME * ALT_HISTORY_SIZE);

unsigned long seconds = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(1000);

	// Setup the io pins
	
  // throttle input pin
  pinMode(INPUT_PIN,INPUT);
  // mode set button
  pinMode(BUTTON_PIN,INPUT);
  // throttle output pin
  pinMode(OUTPUT_PIN,OUTPUT);

	// attach() starts sending pulses right away.  Without the last argument
  // it sends 1500µs (half throttle) until the first write in loop().
  t_out.attach(4, 1000, 2000, 1000);

	// setup the bmp580

	if (!bmp.begin(BMP5XX_ALTERNATIVE_ADDRESS, &Wire)) {
    // For SPI mode (uncomment the line below and comment out the I2C line above):
    // if (!bmp.begin(BMP5XX_CS_PIN, &SPI)) {
    Serial.println(F("Could not find a valid BMP5xx sensor, check wiring!"));
    // The throttle output stays at 1000µs.  Show a solid red led so the
    // failure is visible without a serial connection.
    pixels.begin();
    pixels.setBrightness(20);
    pixels.fill(FOUL_COLOR);
    pixels.show();
    while (1) delay(10);
  }
  
  Serial.println(F("Setting temperature oversampling to 2X..."));
  bmp.setTemperatureOversampling(BMP5XX_OVERSAMPLING_2X);
  Serial.println(F("Setting pressure oversampling to 16X..."));
  bmp.setPressureOversampling(BMP5XX_OVERSAMPLING_16X);
  Serial.println(F("Setting IIR filter to coefficient 3..."));
  bmp.setIIRFilterCoeff(BMP5XX_IIR_FILTER_COEFF_3);
  Serial.println(F("Setting output data rate to 50 Hz..."));
  bmp.setOutputDataRate(BMP5XX_ODR_50_HZ);
  Serial.println(F("Setting power mode to normal..."));
  desiredMode = BMP5XX_POWERMODE_NORMAL;
  bmp.setPowerMode(desiredMode);
  Serial.println(F("Enabling pressure measurement..."));
  bmp.enablePressure(true);
  Serial.println(F("Configuring interrupt pin with data ready source..."));
  bmp.configureInterrupt(BMP5XX_INTERRUPT_LATCHED, BMP5XX_INTERRUPT_ACTIVE_HIGH, BMP5XX_INTERRUPT_PUSH_PULL, BMP5XX_INTERRUPT_DATA_READY, true);



	// initialize the past altitude array
	
	//Add a delay here so that we get a clean measurement
	delay(500);

	// Without the readTemperature call we get bad values for altitude
  bmp.readTemperature(); 

  float cur_altitude = bmp.readAltitude();
  Serial.print("start altitude ");
  Serial.println(cur_altitude);
  start_altitude = cur_altitude;

  for(int i = 0 ; i < ALT_HISTORY_SIZE ; i++) {
		previous_altitude_arr[i]= cur_altitude;
	}

#if defined(NEOPIXEL_POWER)
  // If this board has a power control pin, we must set it to output and high
  // in order to enable the NeoPixels. We put this in an #if defined so it can
  // be reused for other boards without compilation errors
  pinMode(NEOPIXEL_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_POWER, HIGH);
#endif

  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  pixels.setBrightness(20); // not so#if defined(NEOPIXEL_POWER)
  // If this board has a power control pin, we must set it to output and high
  // in order to ena bright


  throttle_low_start = millis();
  state = Wait_For_Valid_Throttle;

  Serial.print("setup done \n");
}


// states
// 0x001  80m ready
// 0x002 100m ready
// 0x004 150m ready
// 0x008  80m alt
// 0x010 100m alt
// 0x020 150m alt
// 0x040  80m time
// 0x080 100m time
// 0x100 150m time
// 0x200 foul


int patterns[] = {
  0b1111111111,
  0b1111111000,
  0b1111111110,
  0b1000000000,
  0b1111000100,
  0b1111000000,
  0b1111000000,
  0b1000000000,
  0b1111111111,
  0b1000000000,
  0b1110110110,
  0b1000000000,
  0b1100100100,
  0b1000000000,
  0b1000000000,
  0b1000000000
};
  
int alt_state = 2;


void blink(uint32_t ms) {
  if (state == Fault) {
    // Solid, unblinking -- unmistakable even without reading the pattern,
    // same convention as the sensor-not-found lockup in setup().
    pixels.fill(FAULT_COLOR);
    pixels.show();
    return;
  }
  uint32_t phase = (ms >> 8) & 15;
  int s = alt_state;
  switch (reason) {
  case Ready:
    s = alt_state;
    break;
  case Altitude:
    s = alt_state + 3;
    break;
  case Time:
    s = alt_state + 6;
    break;
  case Foul:
    s = 9;
    break;
  default:
    s = 0;
  }
  int light = (patterns[phase] >> s) & 1;  
  if (light) {
    pixels.fill(color);
    pixels.show();
  } else {
    pixels.fill(0x000000);
    pixels.show();
  }
}

int button_state = 0;
// This insures if the button is held down we only read it once
int read_button(int pin) {
  int r = digitalRead(pin);
  if (r) {
    if (button_state == 0) {
      button_state = 1;
      return 1;
    } else {
      return 0;
    }
  } else {
    button_state = 0;
    return 0;
  }
}

void loop() {

  int out_value = 1000; // by default we are going to output 1000µs 

  int in_value = pulseIn(2,HIGH,50000);

  // performReading() is what readTemperature()/readAltitude() call
  // internally; calling it ourselves lets us see whether THIS read actually
  // succeeded instead of silently getting a stale cached value back (H1).
  bool sensor_ok = bmp.performReading();
  float cur_altitude = bmp.readAltitude();

  uint32_t now = millis();

  // Sensor fault latch: a few consecutive failed reads means the sensor has
  // actually gone bad (disconnected, bus fault, etc.), not a one-off glitch.
  // Once latched it's permanent for this boot -- cut throttle and show a
  // solid fault-colored LED (see blink()) until the board is power-cycled.
  if (state != Fault) {
    if (sensor_ok) {
      sensor_fail_count = 0;
    } else {
      sensor_fail_count++;
      Serial.println("sensor read failed");
      if (sensor_fail_count >= SENSOR_FAIL_THRESHOLD) {
        state = Fault;
        color = FAULT_COLOR;
        Serial.println("sensor fault -- cutting throttle");
      }
    }
  }

  // Throttle-low interlock: don't allow arming until a valid low throttle
  // has been seen continuously for THROTTLE_LOW_MS after power up.
  if (state == Wait_For_Valid_Throttle) {
    if (in_value < THROTTLE_MIN_VALID_US || in_value > THROTTLE_LOW_US) {
      // If we don't have a valid throttle signal 
      throttle_low_start = now;
    } else if (now - throttle_low_start >= THROTTLE_LOW_MS) {
      state = Can_Arm;
      color = READY_COLOR;
#if DEBUG_OUTPUT 
      Serial.println("throttle low, arming enabled");
#endif
    }
  }
	
	//Do blinking logic
  blink(now);

	// process button input
  if (read_button(11) && (alt_state < 3)) {
    alt_state = (alt_state + 1) % 3;
    altitude = altitude_list[alt_state];
    timer = timer_list[alt_state];
  }

	// compute vertical speed
  float vspd = (cur_altitude - previous_altitude_arr[pa_idx]) * vspd_correction;

#if DEBUG_OUTPUT 

if (0) {
//	if (now / 1000 > seconds ) {
		Serial.print("time ");
		Serial.println(now/1000);
		Serial.println(loop_counter);
		Serial.println(vspd);
		Serial.println(in_value);
		Serial.println(altitude);
		Serial.println(cur_altitude);
    Serial.print ("--------------------- ");
    Serial.println(cur_altitude - start_altitude);
		Serial.println(bmp.readPressure());
		Serial.println(cutoff_altitude);

	}

#endif

  if (state == Armmed) {
    // disarm if above target alt -- checked first: altitude is the more
    // specific, safety-relevant reason, and these must be mutually
    // exclusive (else if) so a pass where both conditions are true doesn't
    // silently overwrite reason = Altitude with reason = Time (H5)
    if (cur_altitude + (vspd * OVERSHOOT_FACTOR)> base_altitude + altitude) {
 #if DEBUG_OUTPUT 
      Serial.println("altitude triggered");
      Serial.println(cur_altitude);
      Serial.println(vspd * OVERSHOOT_FACTOR);
      Serial.println(vspd);
      Serial.println(previous_altitude_arr[pa_idx]);

      Serial.println(base_altitude);
      Serial.println(altitude);
#endif
      state = Done_Cant_Rearm;
      color = DONE_COLOR;
      reason = Altitude;
      cutoff_altitude = cur_altitude;
    } else if (now > base_timer + timer) {
      // disarm if after time
#if DEBUG_OUTPUT 
      Serial.println("timer triggered");
			Serial.print("shutting down at ");
      Serial.println(now);
#endif
      state = Done_Cant_Rearm;
      color = DONE_COLOR;
      reason = Time;
    }
  } else {
    // arm if you can arm and the throttle is above 20%
    if ((state == Can_Arm || state == Done_Can_Rearm) && (in_value > 1200)) {
			// if doing a relight we are not going to reset the base altitude
      if(arm_count == 0) base_altitude = cur_altitude;
      base_timer = now;
#if DEBUG_OUTPUT 
      Serial.println("starting arm");
      Serial.println(base_timer);
      Serial.println(timer);
#endif
      state = Armmed;
      color = ARMED_COLOR;
      arm_count++;
			// If this is a re-arm then we will show the foul signal on the LED
      if (arm_count > 1) {
        color = FOUL_COLOR;
        reason = Foul;
      }
    }
    // If we are not armed and cant arm and we are no more than 20m
    // high re enable arming.  This is assuming that in a contest
    // situation that it would be reasonable to relight to keep the
    // plane from landing too far away
    if (state == Done_Cant_Rearm && 
				((cur_altitude < base_altitude + 20) ||
				 (now > base_timer + timer + timer)) && 
				(in_value < THROTTLE_LOW_US)) {
      state = Done_Can_Rearm;
    }
  }
  if (state == Armmed) {
    out_value = in_value;
  }
	t_out.writeMicroseconds(out_value);

  previous_altitude_arr[pa_idx] = cur_altitude;
  pa_idx = (pa_idx + 1) % ALT_HISTORY_SIZE;
  loop_counter++;
  seconds = now / 1000;
}
