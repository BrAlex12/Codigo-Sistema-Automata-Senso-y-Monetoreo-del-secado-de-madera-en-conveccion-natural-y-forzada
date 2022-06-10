//*****CODIGO DEL SISTEMA AUTOMATA PARA EL SENSADO Y MONITOREO EN EL SECADO DE LA*****
//             *****MADERA EN CONVECCION NATURAL Y CONVECCION FORZADA*****
// Insertar librerias
#include <SD.h>
#include "RTClib.h"
#include "DHT.h"
#include <Servo.h>
#include "LiquidCrystal_I2C.h"
#include "Keypad.h"
#include "max6675.h"
#include <OneWire.h>
#include <DallasTemperature.h>

//******Definir variables de control y nombrarlas*****
#define ONE_WIRE_BUS 12
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

#define SERVO
//#define NANO //en caso de usarse una extrencion con nano

#define YL_FACTOR 1
#define TICK_PERIOD 300
#define SAMPLE_PERIOD 3000
#define LOG_PERIOD 60000
#define SD_CS 53
#define UPPER_SERVO_PIN 7
#define LOWER_SERVO_PIN 6
#define UPPER_SERVO_MIN 0
#define UPPER_SERVO_MAX 9
#define LOWER_SERVO_MIN 0
#define LOWER_SERVO_MAX 9

#ifdef SIMULATE
#define LCD_I2C_DIR 0x20
#define DHTTYPE DHT11
#else
#define LCD_I2C_DIR 0x27
#define DHTTYPE DHT22
#endif

#define DHTPIN0 10
#define DHTPIN1 11
#define DHTPIN2 8
#define DHTPIN3 9
#define DHTPIN4 12

#define DHTMAX 5
#define LMMAX 4
#define YLMAX 4

#define PINREL1 5
#define PINREL2 4

#define ROWS 4
#define COLS 3

#define thermoDO 35
#define thermoCS 37
#define thermoCLK 39
float tempC;
MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

//Delaramos los modos y variables de control con respecto a la logica programable
enum STATES {PANEL, PASS, MENU, MODE, CTRL, TREND} state;
enum MODES {MANUAL, AUTO} mode;
enum MENU_AUTO_STATES {SP_MAX_T, SP_MAX_U, SP_MIN_T, SP_MIN_U, SP_DONE} menu_auto_state;
enum MENU_MANUAL_STATES {U_SERVO_POS, L_SERVO_POS, SERVO_DONE} menu_manual_state;


RTC_DS3231 rtc;

DHT DHTs[DHTMAX] = {DHT(DHTPIN0, DHTTYPE), DHT(DHTPIN1, DHTTYPE), DHT(DHTPIN2, DHTTYPE),
                    DHT(DHTPIN3, DHTTYPE), DHT(DHTPIN4, DHTTYPE)
                   };

//ServoMotor Instances
Servo upper_servo;
Servo lower_servo;

LiquidCrystal_I2C lcd(LCD_I2C_DIR, 20, 4);

//Matriz Keypad configuraciones
char keys[ROWS][COLS] = {
  {'1', '2', '3'},
  {'4', '5', '6'},
  {'7', '8', '9'},
  {'*', '0', '#'}
};
byte row_pins[ROWS] = {22, 23, 24, 25}; //delarar pines para filas de keypad
byte col_pins[COLS] = {26, 27, 28}; //delarar pines para columnas de keypad
Keypad keypad = Keypad( makeKeymap(keys), row_pins, col_pins, ROWS, COLS );

union
{
  float floatingRead;
  char bytes[4];
} sample;


/*******Delaracion de Variables Globales de Sensado*******/
float t[DHTMAX] = {0};
float h[DHTMAX] = {0};
float t_sum[DHTMAX] = {0};
float h_sum[DHTMAX] = {0};
float t_avg[DHTMAX] = {0};
float h_avg[DHTMAX] = {0};
float t_avgGlobal, h_avgGlobal = 0.0;
float lm35[LMMAX] = {0};
float lm35_sum[LMMAX] = {0};
float lm35_avg[LMMAX] = {0};
float lm35_temp = 0.00;
float yl69[YLMAX] = {0};
float yl69_sum[YLMAX] = {0};
float yl69_avg[YLMAX] = {0};
float temp;
float temp_sum;
float temp_avg;
float tempC_sum;
float tempC_avg;
float HMadera;
float HMadera_sum;
float HMadera_avg;
float Hum = 0;
float Sensor;
int  n = 100;

const int lm35_pins[LMMAX] = {A1, A1, A2, A3};
const int yl69_pins[YLMAX] = {A8, A9, A10, A11};

//Declarar las posiciones de los servomotores
int pos_upper_servo;
int pos_lower_servo;

int sample_counter = 0;
long last_sample = 0L;
long last_tick = 0L;

bool logdata = false;
bool fan_state = false;

//Delarar Limites Setpoints para modo Automatico
float sp_min = 40.0;
float sp_max = 45.0;

byte sp_min_tens, sp_min_unit = 0;
byte sp_max_tens, sp_max_unit = 0;
byte pos_upper_servo_u, pos_lower_servo_u = 0;

/*****Declaracion de Funciones y la ejecucion de los actuadores*****/
float f_mean(float *f_array, int SIZE) {
  float sum = 0.0;

  for (int n = 0; n < SIZE; n += 1) {
    sum += f_array[n];
  }

  return sum / float(SIZE);
}

void controlVents(float wood_temp) {
  if (f_mean(t, DHTMAX) < sp_min) {
#ifdef SERVO
    pos_upper_servo = UPPER_SERVO_MIN;
    pos_lower_servo = LOWER_SERVO_MIN;
    upper_servo.write(pos_upper_servo * 20);
    lower_servo.write(pos_lower_servo * 20);
    digitalWrite(PINREL1, LOW);
    digitalWrite(PINREL2, LOW);
#else
    digitalWrite(PINREL1, LOW);
    digitalWrite(PINREL2, LOW);
#endif
  }
  else if (f_mean(t, DHTMAX) > sp_max) {
#ifdef SERVO
    pos_upper_servo = UPPER_SERVO_MAX;
    pos_lower_servo = LOWER_SERVO_MAX;
    upper_servo.write(pos_upper_servo * 20);
    lower_servo.write(pos_lower_servo * 20);
    digitalWrite(PINREL1, HIGH);
    digitalWrite(PINREL2, HIGH);
#else
    digitalWrite(PINREL1, HIGH);
    digitalWrite(PINREL2, HIGH);
#endif
  }
}

bool is_number(char key) {
  if (key - 48 >= 0 && key - 48 <= 9) return true;
  else return false;
}
// Delaramos el menú de transicion de variables
void menu_transition(void) {
  sp_max_unit = (byte)sp_max % 10;
  sp_max_tens = (byte)sp_max / 10;
  sp_min_unit = (byte)sp_min % 10;
  sp_min_tens = (byte)sp_min / 10;
  pos_upper_servo_u = pos_upper_servo;
  pos_lower_servo_u = pos_lower_servo;
}
// Menu inicial de presentacion
void print_panel(bool refresh) {
  if (refresh) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("UNI- PANEL");
    lcd.setCursor(0, 1);
    lcd.print("SP: ");
    lcd.setCursor(17, 1);
    lcd.print("C");
    lcd.setCursor(10, 1);
    lcd.print("TM:");
    lcd.setCursor(0, 2);
    lcd.print("HM: ");
    lcd.setCursor(10, 2);
    lcd.print("VE:");
    lcd.setCursor(0, 3);
    lcd.print("HA: ");
    lcd.setCursor(8, 3);
    lcd.print("%");
    lcd.setCursor(10, 3);
    lcd.print("TA:");
    lcd.setCursor(18, 3);
    lcd.print("C");
  }
  // Menu principal de variables y visualizacion en lcd y monitor serial

  lcd.setCursor(4, 1);
  if (mode == AUTO) {
    lcd.print(sp_min, 0);
    lcd.print("-");
    lcd.print(sp_max, 0);
  } else lcd.print("N/D");
  lcd.setCursor(13, 1);
  lcd.print(tempC);
  lcd.setCursor(3, 2);
  lcd.print(HMadera, 0);
  lcd.setCursor(14, 2);
  //lcd.print(fan_state?"ENC":"APG");
  lcd.print(pos_upper_servo);
  lcd.print('/');
  lcd.print(pos_lower_servo);
  lcd.setCursor(3, 3);
  lcd.print(f_mean(h, DHTMAX), 0);
  lcd.setCursor(13, 3);
  lcd.print(f_mean(t, DHTMAX));
}
// Insertar contraseña de acceso a cofiguraciones
void print_pass(bool refresh) {
  if (refresh) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("PASS");
    lcd.setCursor(0, 2);
    lcd.print("Digite el PIN: ****");
  }
}
// Menu de configuraciones
void print_menu(bool refresh) {
  if (refresh) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("MENU");
    lcd.setCursor(0, 1);
    lcd.print("1 -> MODO");
    lcd.setCursor(0, 2);
    lcd.print("2 -> CONTROL");
    // lcd.setCursor(0,3);
    // lcd.print("3 -> HIST.");
  }
}
// Menu elección modo AUTO/MANUAL
void print_mode(bool refresh) {
  if (refresh) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("MODO");
    lcd.setCursor(1, 1);
    lcd.print("0:MANUAL  1:AUTO");
    lcd.setCursor(1, 3);
    lcd.print("Actual: ");
    lcd.print(mode ? "AUTO" : "MANUAL");
  }
}
// Menu de setpoints de operación
void print_ctrl(bool refresh) {
  if (refresh) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("CONTROL");
  }

  if (mode == AUTO) {
    lcd.setCursor(1, 1);
    lcd.print("A: SP MAX = [    ]");
    lcd.setCursor(15, 1);
    lcd.print(sp_max_tens);
    lcd.print(sp_max_unit);
    lcd.setCursor(1, 2);
    lcd.print("B: SP MIN = [    ]");
    lcd.setCursor(15, 2);
    lcd.print(sp_min_tens);
    lcd.print(sp_min_unit);
  } else {
    lcd.setCursor(0, 1);
    lcd.print("A: Vent A = [   ]");
    lcd.setCursor(14, 1);
    lcd.print(pos_upper_servo_u);
    lcd.setCursor(0, 2);
    lcd.print("B: Vent B = [   ]");
    lcd.setCursor(14, 2);
    lcd.print(pos_lower_servo_u);
  }
}
// Funcion de visualizacion de hora en LCD
void print_hour() {
  DateTime now = rtc.now();

  lcd.setCursor(14, 0);
  if (now.hour() < 10) lcd.print('0');
  lcd.print(now.hour());
  lcd.print(":");
  if (now.minute() < 10) lcd.print('0');
  lcd.print(now.minute());
}
// Delaracion de funcion logica de pantallas e interfaz del sistema
void print_screen(bool F5) {
  switch (state)
  {
    case PANEL:
      print_panel(F5);
      break;

    case PASS:
      print_pass(F5);
      break;

    case MENU:
      print_menu(F5);
      break;

    case MODE:
      print_mode(F5);
      break;

    case CTRL:
      print_ctrl(F5);
      break;

    default:
      break;
  }
  print_hour();
}
// Delaracion de funcion logica de keypad e interfaz del sistema
void state_machine(char sm_key, bool refresh) {
  switch (state) {
    case PANEL:
      if (sm_key == '*') {
        state = PASS;
      }
      else if (sm_key == '#') {
      }
      break;

    case PASS:
      if (sm_key == '*') {
        state = MENU;
      }
      else if (sm_key == '#') {
        state = PANEL;
      }
      break;

    case MENU:
      if (sm_key == '1') {
        state = MODE;
      }
      else if (sm_key == '2') {
        state = CTRL;
        menu_transition();
      }
      // else if(sm_key == '3'){
      //   state = TREND;
      // }
      else if (sm_key == '#') {
        state = PANEL;
      }
      break;
    case MODE:
      if (sm_key == '0') {
        mode = MANUAL;
        state = MENU;
        menu_manual_state = U_SERVO_POS;
      }
      else if (sm_key == '1') {
        mode = AUTO;
        state = MENU;
        menu_auto_state = SP_MAX_T;
      }
      else if (sm_key == '*')  state = PANEL;
      else if (sm_key == '#')  state = MENU;
      break;

    case CTRL:
      if (mode == AUTO) {
        switch (menu_auto_state)
        {
          case SP_MAX_T:
            if (is_number(sm_key)) {
              sp_max_tens = sm_key - 48;
              menu_auto_state = SP_MAX_U;
            }
            break;
          case SP_MAX_U:
            if (is_number(sm_key)) {
              sp_max_unit = sm_key - 48;
              menu_auto_state = SP_MIN_T;
            }
            break;
          case SP_MIN_T:
            if (is_number(sm_key)) {
              sp_min_tens = sm_key - 48;
              menu_auto_state = SP_MIN_U;
            }
            break;
          case SP_MIN_U:
            if (is_number(sm_key)) {
              sp_min_unit = sm_key - 48;
              menu_auto_state = SP_DONE;
            }
            break;
          default:
            break;
        }
        if (sm_key == '*') {
          if ((sp_max_tens * 10 + sp_max_unit) > sp_min_tens * 10 + sp_min_unit) {
            sp_max = sp_max_tens * 10 + sp_max_unit;
            sp_min = sp_min_tens * 10 + sp_min_unit;
          }
          state = PANEL;
          menu_auto_state = SP_MAX_T;
        }
        if (sm_key == '#') {
          state = MENU;
        }
      } else {
        switch (menu_manual_state) {
          case U_SERVO_POS:
            if (is_number(sm_key)) {
              pos_upper_servo_u = sm_key - 48;
              menu_manual_state = L_SERVO_POS;
            }
            break;
          case L_SERVO_POS:
            if (is_number(sm_key)) {
              pos_lower_servo_u = sm_key - 48;
              menu_manual_state = SERVO_DONE;
            }
            break;
          case SERVO_DONE:
            break;
          default:
            break;
        }
        if (sm_key == '*') {
          pos_upper_servo = pos_upper_servo_u;
          pos_lower_servo = pos_lower_servo_u;
          state = PANEL;
        }
        if (sm_key == '#') {
          state = MENU;
        }
        break;

      default:
        break;
      }
  }
  print_screen(refresh);
}
void keypadEvent(KeypadEvent key) {
  if (keypad.getState() == PRESSED) state_machine(key, true);
}
// Declaracion de las funciones bases del sistema e inicializar el sistema y sus compenentes
void setup() {
  Serial.begin( 57600 );   // inicializar monitor serial
  analogReference(INTERNAL2V56);
  if (! rtc.begin()) {     // inicializar RTC (Real time  clock)
    Serial.println(F("Couldn't find RTC"));
    Serial.flush();
    abort();
  }
  if (rtc.lostPower()) {
    Serial.println("RTC lost power, let's set the time!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  DateTime now = rtc.now();

  Serial.println(F("DHT22 test!")); // inicializar DHT 22
  for (int i = 0; i < DHTMAX; i += 1) {
    DHTs[i].begin();
    Serial.print("begin DHT ");
    Serial.println(i);
  }
  Serial.print("Initializing SD card..."); // inicializar modulo de tarjeta SD
  if (!SD.begin(SD_CS)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1);
  }
  Serial.println("card initialized.");

  upper_servo.attach(UPPER_SERVO_PIN);// inicializar Servo motores
  lower_servo.attach(LOWER_SERVO_PIN);// inicializar Servo motores
  pinMode(PINREL1, OUTPUT);
  pinMode(PINREL2, OUTPUT);

  sensors.begin(); // inicializar sensores
  lcd.init();                      // inicializar LCD
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print(F("Universidad Nacional"));
  lcd.setCursor(0, 1);
  lcd.print(F(" de Ingenieria - UNI"));
  lcd.setCursor(6, 2);
  if (now.hour() < 10) lcd.print('0');
  lcd.print(now.hour());
  lcd.print(":");
  if (now.minute() < 10) lcd.print('0');
  lcd.print(now.minute());
  Serial.println("Iniciando...");
  delay(1000);
  lcd.clear();
  keypad.addEventListener(keypadEvent);
  print_screen(true);

}
// Definir funciones ciclicas del sistema
void loop() {
  long timestamp;
  char hourBuffer[] = "hh:mm";
  char dateBuffer[] = "MM-DD-YYYY";
  String dataString = "";

  char key = keypad.getKey();

  if (key) {
    Serial.println(key);
  }

  if (millis() - last_tick > TICK_PERIOD) { // definir el tiempo de guardado de datos
    DateTime now = rtc.now();
    timestamp = now.unixtime();

    if (now.minute() % 2 == 0 && now.second() == 0 && logdata == false && sample_counter != 0) {
      logdata = true;

      for (int i = 0; i < DHTMAX; i += 1) {
        t_avg[i] = t_sum[i] / sample_counter;
        h_avg[i] = h_sum[i] / sample_counter;
      }
      // for (int i = 0; i < LMMAX; i += 1) {            //LM35 deshabilitado por no uso en el sistema final
      // lm35_avg[i] = lm35_sum[i] / sample_counter;
      // }
      for (int i = 0; i < YLMAX; i += 1) {
        yl69_avg[i] = yl69_sum[i] / sample_counter;
      }
      temp_avg = temp_sum / sample_counter;
      tempC_avg = tempC_sum / sample_counter;
      HMadera_avg = HMadera_sum / sample_counter;
      sample_counter = 0;
      memset(t_sum, 0, sizeof(t_sum));
      memset(h_sum, 0, sizeof(h_sum));
      // memset(lm35_sum, 0, sizeof(lm35_sum));
      memset(yl69_sum, 0, sizeof(yl69_sum));
      Serial.println("-------");
      Serial.print(f_mean(t_avg, DHTMAX));
      Serial.println(" C");
      Serial.print(f_mean(h_avg, DHTMAX));
      Serial.println(" %");
      //Serial.print(f_mean(lm35_avg, LMMAX));        //En caso de usar se el sensor Lm35
      //Serial.println(" C");
      //Serial.print(f_mean(yl69_avg, YLMAX));
      //Serial.println(" % ");
      Serial.print(temp_avg);
      Serial.println(" C");
      Serial.print(tempC_avg);
      Serial.println(" C");
      Serial.print(HMadera_avg);
      Serial.println(" %");
      Serial.println("-------");

      dataString += String(timestamp);
      dataString += ",";
      dataString += now.toString(dateBuffer);
      dataString += ",";
      dataString += now.toString(hourBuffer);
      dataString += ",";
      for (byte i = 0; i < DHTMAX; i++)
      {
        dataString += String(t_avg[i]);
        dataString += ",";
        dataString += String(h_avg[i]);
        dataString += ",";
      }
      //      for (byte i = 0; i < LMMAX; i++)         //En caso de usar se el sensor Lm35
      //      {
      //        dataString += String(lm35_avg[i]);
      //        dataString += ",";
      //      }
      //      for (byte i = 0; i < YLMAX; i++)
      //      {
      //        dataString += String(yl69_avg[i]);
      //        dataString += ",";
      //      }
      //  dataString += String(yl69_avg[i]);
      // dataString += ",";

      File dataFile = SD.open("DATALOG.txt", FILE_WRITE);  // si la SD esta disponible se apertura archivo de texto y guardara los datos sensados
      if (dataFile) {
        dataFile.print(dataString);
        dataFile.print(temp_avg);
        dataFile.print(",");
        dataFile.print(tempC_avg);
        dataFile.print(",");
        dataFile.println(HMadera_avg);
        dataFile.close();
        // print to the serial port too:
        Serial.print(dataString);
        Serial.print(temp);
        Serial.print(",");
        Serial.print(tempC);
        Serial.print(",");
        Serial.println(HMadera);
        temp_sum = 0;
        tempC_sum = 0;
        HMadera_sum = 0;
      }
      else {
        Serial.println("error opening datalog.txt"); // si el archivo presenta un error aparecera el siguiente mensaje:
      }
    }
    else if (now.minute() % 2 == 1 && now.second() == 0 && logdata == true) {
      logdata = false;
    }
  }
  if (millis() - last_sample > SAMPLE_PERIOD) {
    last_sample = millis();
    sensors.requestTemperatures();

    for ( int i = 0; i < DHTMAX; i += 1 ) {
      t[i] = DHTs[i].readTemperature();
      h[i] = DHTs[i].readHumidity();
      if (isnan(t[i]) || isnan(h[i]))
      {
        return;
      }
      t_sum[i] += t[i];
      h_sum[i] += h[i];
    }
    Serial.print(f_mean(t, DHTMAX));//Se obtiene la temperatura en ºC
    Serial.print("°C | ");
    Serial.print(f_mean(h, DHTMAX));//Se obtiene la humedad ambiente en %
    Serial.print("% | ");
    temp = sensors.getTempCByIndex(0); //Se obtiene la temperatura en ºC
    Serial.print(sensors.getTempCByIndex(0));
    temp_sum += temp;
    Serial.print(temp);
    Serial.print("°C| ");
    tempC = thermocouple.readCelsius(); //Se obtiene la temperatura en ºC
    tempC_sum += tempC;
    Serial.print(tempC);
    Serial.print(" °C| ");
    for (int i = 0; i < n; i++) {
      Sensor = analogRead(A3); // lee el sensor analogico de medir humedad madera
      Hum = (float)Hum + Sensor;
      delay(1);
    }
    Hum = Hum / n;
    HMadera = Hum;
    HMadera_sum += HMadera;// para medir humedad madera se realiza una funcion donde recolecta el promedio de un numero definidos de datos
    Serial.print(HMadera);
    Serial.println(" %|");
    sample_counter += 1;
    if (mode == AUTO)    controlVents(f_mean(t, DHTMAX));//si el sistema esta en modo AUTO  definir los grados de mobilidad de los servomotores y el control de los ventiladores
    else {
#ifdef SERVO
      upper_servo.write(pos_upper_servo * 20);
      lower_servo.write(pos_lower_servo * 20);
      digitalWrite(PINREL1, pos_upper_servo ? HIGH : LOW);
      digitalWrite(PINREL2, pos_lower_servo ? HIGH : LOW);
#else
      digitalWrite(PINREL1, pos_upper_servo ? HIGH : LOW);
      digitalWrite(PINREL2, pos_lower_servo ? HIGH : LOW);
#endif
    }
    print_screen(false);
  }
}
