#pragma region definitions
#include <LCD_I2C.h>
#include <AccelStepper.h>
#include <HCSR04.h>

#define MOTOR_INTERFACE_TYPE 4

#define BUZZER_PIN 2

#define LED_PIN_RED 3
#define LED_PIN_GREEN 4
#define LED_PIN_BLUE 5

#define IN_1 8
#define IN_2 9
#define IN_3 10
#define IN_4 11

#define TRIGGER_PIN 6
#define ECHO_PIN 7

LCD_I2C lcd(0x27, 16, 2);
AccelStepper moteur(MOTOR_INTERFACE_TYPE, IN_1, IN_3, IN_2, IN_4);
HCSR04 hc(TRIGGER_PIN, ECHO_PIN);

enum AppState { NORMAL,
                TROP_PRES,
                TROP_LOIN,
                ALERTE };

AppState appState = NORMAL;

char lcdBuff[2][16] = {
  "                ",
  "                "
};

unsigned long current_time = 0;

int frequency = 1;

int red;
int green;
int blue;

int distance;
int min_distance = 30;
int max_distance = 60;
int distance_alerte = 15;

int angle;
int min_angle = 10;
int max_angle = 170;
float tour = 2038;

float max_step = (tour * max_angle) / 360;
float min_step = (tour * min_angle) / 360;
#pragma endregion

#pragma region etats
int normal_state() {
  static bool first_time = true;
  static int last_position = 0;

  // if (first_time) {
  //   first_time = false;
  // }

  float current_position = map(distance, min_distance, max_distance, min_step, max_step);
  current_position = constrain(current_position, min_step, max_step);

  angle = map(current_position, min_step, max_step, min_angle, max_angle);

  moteur.moveTo(current_position);
  moteur.run();

  if (moteur.distanceToGo() == 0) {
    moteur.disableOutputs();
  } else {
    moteur.enableOutputs();
  }

  bool transition_A = distance <= distance_alerte;
  bool transition_T_P = distance < min_distance && (moteur.distanceToGo() == 0);
  bool transition_T_L = distance > max_distance && (moteur.distanceToGo() == 0);

  if (transition_A) {
    first_time = true;
    appState = ALERTE;
  } else if (transition_T_P) {
    first_time = true;
    appState = TROP_PRES;
  } else if (transition_T_L) {
    first_time = true;
    appState = TROP_LOIN;
  }

  return angle;
}

void trop_pres_state() {
  bool transition_A = distance <= distance_alerte;
  bool transition_N = distance > min_distance;

  if (transition_A) {
    appState = ALERTE;
  } else if (transition_N) {
    appState = NORMAL;
  }
}

void trop_loin_state() {
  bool transition = distance < max_distance;

  if (transition) {
    appState = NORMAL;
  }
}

void alerte_state(unsigned long ct) {
  static unsigned long start_timer = 0;
  static bool timer_started = false;
  const long timer_interval = 3000;

  moteur.disableOutputs();

  bool transition = distance > distance_alerte;
  bool cancel_transition = distance <= distance_alerte;

  tone(BUZZER_PIN, frequency);
  led_blink_task(ct);

  if (transition) {        // detecte condition sortie etat
    if (!timer_started) {  // demarre le timer si pas deja demarre
      start_timer = ct;
      timer_started = true;
    } else if (ct - start_timer >= timer_interval) {  // change detat si le timer est supperieur a 3 secondes
      noTone(BUZZER_PIN);
      set_color_task(0, 0, 0);
      timer_started = false;
      appState = TROP_PRES;
    }
  } else {  // reinitialise le timer si la transition tombe a false
    timer_started = false;
  }
}

void state_manager(unsigned long ct) {
  switch (appState) {
    case NORMAL:
      normal_state();
      break;

    case TROP_PRES:
      trop_pres_state();
      break;

    case TROP_LOIN:
      trop_loin_state();
      break;

    case ALERTE:
      alerte_state(ct);
      break;
  }
}
#pragma endregion

#pragma region tasks
int distance_task(unsigned long ct) {
  static unsigned long previous_time = 0;
  const long interval = 50;
  static int last_distance;

  if (ct - previous_time >= interval) {
    distance = hc.dist();

    previous_time = ct;

    if (distance > 0) {
      last_distance = distance;
    } else {
      distance = last_distance;
    }
  }

  return distance;
}

void print_task(unsigned long ct) {
  static unsigned long previous_time = 0;
  const long interval = 100;

  if (ct - previous_time >= interval) {
    previous_time = ct;

    Serial.print("etd:2255309,dist:");
    Serial.print(distance);
    Serial.print(",deg:");

    lcd.clear();
    snprintf(lcdBuff[0], sizeof(lcdBuff[0]), "Dist : %d cm", distance);
    lcd.setCursor(0, 0);
    lcd.print(lcdBuff[0]);

    lcd.setCursor(0, 1);
    lcd.print("Obj  : ");

    if (distance < min_distance) {
      if (distance < distance_alerte) {
        snprintf(lcdBuff[1], sizeof(lcdBuff[1]), "ALERTE");
        Serial.println("ALERTE");
      } else {
        snprintf(lcdBuff[1], sizeof(lcdBuff[1]), "Trop pres");
        Serial.println("Trop pres");
      }
    } else if (distance > max_distance) {
      snprintf(lcdBuff[1], sizeof(lcdBuff[1]), "Trop loin");
      Serial.println("Trop loin");
    } else {
      snprintf(lcdBuff[1], sizeof(lcdBuff[1]), "%d deg", angle);
      Serial.println(angle);
    }

    lcd.print(lcdBuff[1]);
  }
}

void set_color_task(int R, int G, int B) {
  digitalWrite(LED_PIN_RED, R);
  digitalWrite(LED_PIN_GREEN, G);
  digitalWrite(LED_PIN_BLUE, B);
}

int current_color = 0;
void led_blink_task(unsigned long ct) {
  static unsigned long previous_time = 0;
  const long led_interval = 200;

  if (ct - previous_time >= led_interval) {
    previous_time = ct;

    switch (current_color) {
      case 0:
        set_color_task(255, 0, 0);  // rouge
        current_color++;
        break;
      case 1:
        set_color_task(0, 0, 255);  // bleu
        current_color = 0;
        break;
    }
  }
}
#pragma endregion

#pragma region setup - loop
void setup() {
  Serial.begin(115200);

  pinMode(BUZZER_PIN, OUTPUT);

  pinMode(LED_PIN_RED, OUTPUT);
  pinMode(LED_PIN_GREEN, OUTPUT);
  pinMode(LED_PIN_BLUE, OUTPUT);

  lcd.begin();
  lcd.backlight();

  moteur.setMaxSpeed(500);
  moteur.setAcceleration(100);
  moteur.setSpeed(500);
  moteur.setCurrentPosition(0);

  start_task();
}

void loop() {
  current_time = millis();

  distance_task(current_time);
  state_manager(current_time);
  print_task(current_time);
}
#pragma endregion

#pragma region start_task
void start_task() {
  lcd.setCursor(0, 0);
  lcd.print("2255309");

  lcd.setCursor(0, 1);
  lcd.print("Labo 4B");

  delay(2000);
  lcd.clear();
}
#pragma endregion