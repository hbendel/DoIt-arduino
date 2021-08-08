//#include <Wire.h>
#include <RTClib.h>
#include <LiquidCrystal.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>

//Iniciando la clase Helios
#include <Helios.h>

//Iniciando la libreria Reloj
RTC_DS3231 rtc;

// inicializa la librería lcd con los números de pins de la interfase
const int rs = 47, en = 43, d4 = 41, d5 = 39, d6 = 37, d7 = 35;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

//Iniciando la libreria BlueTooth
SoftwareSerial miBT(1, 0, false); //Crea conexion al bluetooth - PIN 2 a TX y PIN 3 a RX

char NOMBRE[21] = "SkyConqueror v1.0"; // Nombre de 20 caracteres maximo
char BPS = '4';                        // 1=1200 , 2=2400, 3=4800, 4=9600, 5=19200, 6=38400, 7=57600, 8=115200
char PASS[5] = "0000";                 // PIN O CLAVE de 4 caracteres numericos
char DATO = 0;
char command = '0';
char ledRojo = 0;

int LED = 33;
int ESTADO = LOW;
Helios helios;
/////////////////////////////////////////////////
//Declaring Variables - User Input
//////////////////////////////////////////////////
//Helistato = 1 Solar panel = 0
int heliostat = 1;
boolean heliostatActive = LOW;
//Put your latitude and longitude here
float latitude = -34.5343378;
float longitude = -58.4975102;

//////////////////////////////////////////////////
//Declaring Variables
//////////////////////////////////////////////////
//Your Heliostat target, with respect to helios's coordinates
float targetalt = 90;
float targetaz = 90;

double dAzimuth;                   //The sun's azimuth angle
double dElevation;                 // The sun's elevation/altitude angle
double offset_Elv = 90;            //The angular offset to center the Helios's range of motion with the sun's range of motion
double offset_Az = 270;            //The angular offset to center the Helios's range of motion with the sun's range of motion.
float altcommand;                  //The helios's commanded altitute/elevation angle
float azcommand;                   //The helios's commanded azimuch angle
float altitude, azimuth, delta, h; //Some variables used in functions later

//////////////////////////////////////////////////
//Attaching Step Motor
//////////////////////////////////////////////////
//RAMPS 1.4 MOTOR X
#define x_paso A0 // Define el Pin de STEP para Motor de eje X
#define x_dire A1 // Define el Pin de DIR  para Motor de eje X
#define x_habi 38 // Define el Pin de ENABLE para Motor de eje X
//RAMPS 1.4 MOTOR Y
#define y_paso A6
#define y_dire A7
#define y_habi A2

int retardo = 5000;         // Menor numero el giro es mas rapido, mas lento es mas preciso
int steps = 200;            // Cuantos pasos requiere el motor para dar una vuelta
int steps_min = steps / 10; // Pasos minimos requeridos para mover el motor (Ej: 10% de los pasos)
int steps_x;
int steps_y;
int steps_anterior_x;
int steps_anterior_y;
int steps_from_origin_x;
int steps_from_origin_y;

int steps_limiteHigh_x = 100; // Limite en 180º
int steps_limiteLow_x = 0;
int steps_limiteHigh_y = 180;
int steps_limiteLow_y = 50; //Limite en 25º ;

//const int PULSADOR = 31;

//Configuracion del LCD
#define COUNT(x) sizeof(x) / sizeof(*x)
const byte rowsLCD = 2;                                                                 // Filas del LCD
const byte columnsLCD = 16;                                                             // Columnas del LCD
const byte iARROW = 0;                                                                  // ID icono flecha
const byte bARROW[] = {B00000, B00100, B00110, B11111, B00110, B00100, B00000, B00000}; // Bits icono flecha

enum Button
{
  Left,
  Right,
  Up,
  Down,
  Ok,
  Unknown
} btnPressed; // Enumerador con los diferentes botones disponibles
enum Screen
{
  Menu1,
  Menu2,
  Flag,
  Number
};                                                                                                                                                        // Enumerador con los distintos tipos de submenus disponibles
const char *txtMENU[] = {"Modo", "Target X", "Target Y", "Mostrar Tiempo", "Hora Encendido", "Hora Apagado", "Unid. Tiempo", "Guardar y salir", "Salir"}; //Menu principal
const byte iMENU = COUNT(txtMENU);                                                                                                                        // Numero de items/opciones del menu principal
enum eSMENU1
{
  PanelSolar,
  Heliostato,
  Manual
};                                                                            // Enumerador de las opciones disponibles del submenu 2 (tienen que seguir el mismo orden que los textos)
const char *txtSMENU1[] = {" Panel Solar ", "  Heliostato ", "    Manual  "}; // SubMenu 1
enum eSMENU2
{
  FullDate,
  Hours,
  Minutes,
  Seconds,
  Milliseconds
};                                                                                                                    // Opciones disponibles del submenu 1 (tienen que seguir el mismo orden que los textos)
const char *txtSMENU2[] = {" Fecha y Hora ", "    Horas     ", "   Minutos    ", "   Segundos   ", " Milisegundos "}; // SubMenu

/* ESTRUCTURAS CONFIGURACION */
// Estructura STRUCT con las variables que almacenaran los datos que se guardaran en la memoria EEPROM
struct MYDATA
{
  int initialized;
  int mode;
  int target_x;
  int target_y;
  int time_show;
  int time_on;
  int time_off;
  int time_unit;
};

union MEMORY
{ // Estructura UNION para facilitar la lectura y escritura en la EEPROM de la estructura STRUCT
  MYDATA d;
  byte b[sizeof(MYDATA)];
} memory;

unsigned long lastFire = 0;

// PINS ON THE ARDUINO.
// MAKE THESE "CONST INT" VARIABLES AS THESE DO NOT CHANGE IN THE PROGRAM.
const int leftButtonPin = 31;  // Pin 5 for "Left" command
const int rightButtonPin = 29; // Pin 6 for "Right" command
const int upButtonPin = 27;    // Pin 7 for "Up" command
const int downButtonPin = 25;  // Pin 8 for "Down" command
const int enterButtonPin = 23; // Pin 9 for "Enter" command
const int clearButtonPin = 17; // Pin 10 for "Clear" command

// NUMBER COUNT OF BUTTON PRESSES AND COUNTER UNITS.
// MAKE THESE A "INT" VARIABLES TO ALLOW FOR NEGATIVE INTEGERS.
int setting1Counter = 0; // Counters for settings 1 - 5
int setting2Counter = 0;
int setting3Counter = 0;
int setting4Counter = 0;
int setting5Counter = 0;
int directionPush = 0;  // This counter changes the menu option with each "left" or "right" button push.
int upPressCount = 0;   // This counter measures the amount of times the user pushes the "up" button.
int downPressCount = 0; // This counter measures the amount of times the user pushes the "down" button.

// BUTTON PRESS STATES FOR EACH FUNCTION, ALL SET TO "LOW".
// MAKE THESE "BOOLEAN" VARIABLES AS THESE ONLY WILL BE "HIGH" OR "LOW".
boolean buttonStateLeft = LOW; // Button states for the "Left" command
boolean lastButtonStateLeft = LOW;
boolean currentButtonStateLeft = LOW;
boolean buttonStateRight = LOW; // Button states for the "Right" command
boolean lastButtonStateRight = LOW;
boolean currentButtonStateRight = LOW;
boolean buttonStateUp = LOW; // Button states for the "Up" command
boolean lastButtonStateUp = LOW;
boolean currentButtonStateUp = LOW;
boolean buttonStateDown = LOW; // Button states for the "Down" command
boolean lastButtonStateDown = LOW;
boolean currentButtonStateDown = LOW;
boolean buttonStateEnter = LOW; // Button states for the "Enter" command
boolean lastButtonStateEnter = LOW;
boolean currentButtonStateEnter = LOW;
boolean buttonStateClear = LOW; // Button states for the "Clear" command
boolean lastButtonStateClear = LOW;
boolean currentButtonStateClear = LOW;

// DEBOUNCE VARIABLES TO MEASURE THE DEBOUNCING TIME OF A BUTTON PUSH.
// MAKE THESE "UNSIGNED LONG" VARIABLES AS THE NUMERICAL VALUE WILL HAVE AN EXTENDED SIZE.
unsigned long lastDebounceTime = 0; // This variable takes a "snapshot" of time when any button is pushed.
unsigned long debounceDelay = 50;   // Delay time in milliseconds; the amount of time the button is pressed must be higher than the delay to register a push.
DateTime fecha;

void setup()
{
  Serial.begin(9600);
  Serial.println('Serial started at 9600');
  miBT.begin(9600);
  Serial.println("miBT started at 9600");
  //Wire.begin();
  //Serial.begin(9600);
  //pinMode(PULSADOR, INPUT);
  pinMode(LED, OUTPUT);

  //MOTOR X
  pinMode(x_paso, OUTPUT);
  pinMode(x_dire, OUTPUT);
  pinMode(x_habi, OUTPUT);

  //MOTOR Y
  pinMode(y_paso, OUTPUT);
  pinMode(y_dire, OUTPUT);
  pinMode(y_habi, OUTPUT);

  //BOTONERA
  pinMode(leftButtonPin, INPUT);  // SETS THE leftButtonPin AS AN INPUT
  pinMode(rightButtonPin, INPUT); // SETS THE rightButtonPin AS AN INPUT
  pinMode(upButtonPin, INPUT);    // SETS THE upButtonPin AS AN INPUT
  pinMode(downButtonPin, INPUT);  // SETS THE downButtonPin AS AN INPUT
  pinMode(enterButtonPin, INPUT); // SETS THE enterButtonPin AS AN INPUT
  pinMode(clearButtonPin, INPUT); // SETS THE clearButtonPin AS AN INPUT

  //LED
  digitalWrite(LED, HIGH); // Enciende el LED 13 durante 4s antes de configurar el miBTtooth
  delay(4000);
  digitalWrite(LED, LOW); // Apaga el LED 13 para iniciar la programacion

  //configureCommon(); // Setup pins for interrupt
  if (!rtc.begin())
  {
    Serial.println("MODULO RTC no encontrado!");
    while (1)
      ;
  }
  //Fijar Fecha y Hora
  //rtc.adjust(DateTime(__DATE__, __TIME__));

  // Carga la configuracion de la EEPROM, y la configura por primera vez:
  readConfiguration();

  //Setting Bloutooth
  //miBTConfiguration();
  lcdSetup();
  scanBaudrate();
  delay(3000);
}

void loop()
{
  fecha = rtc.now();
  if ((fecha.hour() >= memory.d.time_on) && (fecha.hour() <= memory.d.time_off))
  { //Si estoy entre la hora de encendido y apagado
    heliostatActive = HIGH;
  }
  else
  {
    heliostatActive = LOW;
  }

  static unsigned long tNow = 0;
  static unsigned long tPrevious = 0;

  tNow = millis();
  btnPressed = readButtons();

  if (btnPressed == Button::Ok)
    openMenu();
  // Pinta la pantalla principal cada 1 segundo:
  if (tNow - tPrevious >= 1000)
  {
    tPrevious = tNow;
    if (memory.d.time_show == 1)
    {
      switch (memory.d.time_unit)
      {
      case eSMENU2::FullDate:
        lcd.setCursor(0, 0);
        printDigits(fecha.day());
        lcd.print("/");
        printDigits(fecha.month());
        lcd.print("/");
        lcd.print(fecha.year());
        lcd.print(" ");
        printDigits(fecha.hour());
        lcd.print(":");
        printDigits(fecha.minute());
        break;
      case eSMENU2::Hours:
        lcd.print(tNow / 1000 / 60 / 60);
        lcd.print(" Hor");
        break;
      case eSMENU2::Minutes:
        lcd.print(tNow / 1000 / 60);
        lcd.print(" Min");
        break;
      case eSMENU2::Seconds:
        lcd.print(tNow / 1000);
        lcd.print(" Seg");
        break;
      case eSMENU2::Milliseconds:
        lcd.print(tNow);
        lcd.print(" Mil");
        break;
      }
    }

    // if (memory.d.temp_show == 1)
    // {
    //   lcd.setCursor(memory.d.temp_x, memory.d.temp_y);
    //   switch (memory.d.temp_unit)
    //   {
    //   case eSMENU2::GradeC:
    //     //lcd.print(getTemp());
    //     lcd.print(" C");
    //     break;
    //   case eSMENU2::GradeF:
    //     //lcd.print(1.8 * getTemp() + 32);
    //     lcd.print(" F");
    //     break;
    //   }
    // }
    if (heliostatActive == LOW)
    {
      lcd.setCursor(0, 1);
      lcd.print("   En reposo   ");
    }
    else
    {
      lcd.setCursor(0, 1);
      lcd.print("Az");
      lcd.print(azcommand);
      lcd.print(" ");
      lcd.print("Al");
      lcd.print(altcommand);
    }

    //command = '0';
    readBluetoothButtons();

    digitalWrite(LED, ledRojo);
    /*

    while (miBT.available() > 0 ) {
    DATO = miBT.read();
    Serial.print(DATO);
    if(DATO=='A'){
      digitalWrite(LED, HIGH);
    }

    if(DATO=='a'){
     //digitalWrite(LED, LOW);
    }
    }
  */

    //digitalWrite(LED, !digitalRead(LED)); // cuando termina de configurar el Bluetooth queda el LED 13 parpadeando
  }

  ////////////////////////
  //Posicion del sol
  /////////////////
  if (heliostatActive == HIGH)
  {
    helios.calcSunPos(fecha.year(), fecha.month(), fecha.day(), fecha.hour(), fecha.minute(), fecha.second(), latitude, longitude);

    // En astronomía, el azimut es el ángulo o longitud de arco medido sobre el horizonte celeste que forman el punto cardinal Norte y la proyección vertical del astro
    // https://es.wikipedia.org/wiki/Acimut
    dAzimuth = helios.dAzimuth;
    //show("Sun Position Azimuth", dAzimuth, true);

    // La elevación solar es el ángulo de elevación del Sol. Esto es, el ángulo entre la dirección del sol y el horizonte ideal.
    // https://es.wikipedia.org/wiki/Elevaci%C3%B3n_solar
    dElevation = helios.dElevation;
    //show("Sun Position Elevation", dElevation, true);

    //////////////////////////////////////////////////
    //Act like solar panel
    //////////////////////////////////////////////////
    switch (memory.d.mode)
    {
    case eSMENU1::PanelSolar:
      Serial.println("Modo Panel Solar ");
      steps_anterior_x = steps_from_origin_x;
      steps_anterior_y = steps_from_origin_y;
      azcommand = offset_Az - dAzimuth;     //So that the servo's range of motion is aligned with the sun's visable range of motion
      altcommand = offset_Elv - dElevation; //So that the servo's range of motion is aligned with the sun's visable range of motion

      steps_from_origin_x = ((azcommand * steps) / 360);
      steps_from_origin_y = ((altcommand * steps) / 360);
      steps_x = steps_from_origin_x - steps_anterior_x;
      steps_y = steps_from_origin_y - steps_anterior_y;

      if (steps_x >= steps / 10)
      {
        giro(x_paso, x_dire, x_habi, steps_x);
        //altServoMotor.write(altcommand); //Command the altitude servo to the reported position
        Serial.println(" ");
        Serial.print("Commanded Azimuth =");
        //Serial.println( altServoMotor.read()); //Print the angle the servo moved to
        Serial.println(azcommand);
        delay(1000); //Give the Altitude servo time to move to position, so the source voltage doesn't drop below the 4.5 volt limit of the servo
      }
      if (steps_y >= steps / 10)
      {
        giro(y_paso, y_dire, y_habi, steps_y);
        //ServoMotor.write(azcommand); //Command the azimuth servo to the reported position
        //Serial.println(azServoMotor.read()); //Print the angle the servo moved to
        Serial.print("Commanded Elevation =");
        Serial.println(altcommand);
        delay(1000); //Give the Azimuth servo time to move to position, so the source voltage doesn't drop below the 4.5 volt limit of the servo
      }
      break;
    case eSMENU1::Heliostato:
      //Serial.println("Modo Heliostato");
      steps_anterior_x = steps_from_origin_x;
      steps_anterior_y = steps_from_origin_y;
      // Serial.print("Step_anterior_x = ");
      // Serial.print(steps_anterior_x);
      // Serial.print(" Steps X = ");
      // Serial.println(steps_x);
      // Serial.print("Step_anterior_y = ");
      // Serial.print(steps_anterior_y);
      // Serial.print(" Steps Y = ");
      // Serial.println(steps_y);
      targetaz = memory.d.target_x;  //So that the servo's range of motion is aligned with the sun's visable range of motion
      targetalt = memory.d.target_y; //So that the servo's range of motion is aligned with the sun's visable range of motion
     
      azcommand = FindHeliostatAngle(dElevation, dAzimuth,targetaz, targetalt, 2);  //Shifted so that the servo's zero is centered on the coordinate systems values for of 180degrees
      altcommand = FindHeliostatAngle(dElevation, dAzimuth, targetaz, targetalt, 1); //Shifted so that the servo's zero is aligned with the coord. sys zero
      steps_from_origin_x = ((azcommand * steps) / 360);
      steps_from_origin_y = ((altcommand * steps) / 360);
      steps_x = steps_from_origin_x - steps_anterior_x;
      steps_y = steps_from_origin_y - steps_anterior_y;
      if (steps_x >= steps / 10)
      {
        giro(x_paso, x_dire, x_habi, steps_x);
        Serial.print("Commanded Azimuth = ");
        Serial.print(azcommand);
        // Serial.println( altServoMotor.read()); //Print the angle the servo was told to move to
        delay(1000); //Give the Altitude servo time to move to position, so the source voltage doesn't drop below 4.5 volts
      }
      if (steps_y >= steps / 10)
      {
        giro(y_paso, y_dire, y_habi, steps_y);
        Serial.print("Commanded Elevation = ");
        Serial.print(altcommand);

        // Serial.println(azServoMotor.read()); //Print the angle the servo moved to
        Serial.println(" ");
        delay(1000); //Give the Altitude servo time to move to position, so the source voltage doesn't drop below 4.5 volts
      }
      break;
    case eSMENU1::Manual:
      //Serial.println(" ");
      //Serial.println("Modo Manual");
      steps_anterior_x = steps_from_origin_x;
      steps_anterior_y = steps_from_origin_y;
      //steps_anterior_x = 0;
      //azcommand = 90;
      azcommand = memory.d.target_x;  //So that the servo's range of motion is aligned with the sun's visable range of motion
      altcommand = memory.d.target_y; //So that the servo's range of motion is aligned with the sun's visable range of motion
      steps_from_origin_x = ((azcommand * steps) / 360);
      steps_from_origin_y = ((altcommand * steps) / 360);
      steps_x = steps_from_origin_x - steps_anterior_x;
      steps_y = steps_from_origin_y - steps_anterior_y;
      // Serial.print("steps_x =");
      // Serial.print(steps_x);
      // Serial.print(" steps_y =");
      // Serial.println(steps_y);
      if (steps_x >= steps / 10)
      {
        giro(x_paso, x_dire, x_habi, steps_x);
        //altServoMotor.write(altcommand); //Command the altitude servo to the reported position
        //Serial.println( altServoMotor.read()); //Print the angle the servo moved to
        delay(1000); //Give the Altitude servo time to move to position, so the source voltage doesn't drop below the 4.5 volt limit of the servo
      }
      if (steps_y >= steps / 10)
      {
        giro(y_paso, y_dire, y_habi, steps_y);
        //ServoMotor.write(azcommand); //Command the azimuth servo to the reported position
        //Serial.println(azServoMotor.read()); //Print the angle the servo moved to
        delay(1000); //Give the Azimuth servo time to move to position, so the source voltage doesn't drop below the 4.5 volt limit of the servo
      }
      break;
    }
  }
  delay(100);
}

/**
    MUESTRA EL MENU PRINCIPAL EN EL LCD.
*/
void openMenu()
{
  byte idxMenu = 0;
  boolean exitMenu = false;
  boolean forcePrint = true;

  lcd.clear();

  while (!exitMenu)
  {
    btnPressed = readButtons();

    if (btnPressed == Button::Up && idxMenu - 1 >= 0)
    {
      idxMenu--;
    }
    else if (btnPressed == Button::Down && idxMenu + 1 < iMENU)
    {
      idxMenu++;
    }
    else if (btnPressed == Button::Ok)
    {

      switch (idxMenu)
      {
      case 0:
        openSubMenu(idxMenu, Screen::Menu1, &memory.d.mode, 0, COUNT(txtSMENU1) - 1);
        break;
      case 1:
        openSubMenu(idxMenu, Screen::Number, &memory.d.target_x, 0, 360);
        break;
      case 2:
        openSubMenu(idxMenu, Screen::Number, &memory.d.target_y, 30, 180);
        break;
      case 3:
        openSubMenu(idxMenu, Screen::Flag, &memory.d.time_show, 0, 1);
        break;
      case 4:
        openSubMenu(idxMenu, Screen::Number, &memory.d.time_on, 0, 23);
        break;
      case 5:
        openSubMenu(idxMenu, Screen::Number, &memory.d.time_off, 0, 23);
        break;
      case 6:
        openSubMenu(idxMenu, Screen::Menu2, &memory.d.time_unit, 0, COUNT(txtSMENU2) - 1);
        break;
      case 7:
        writeConfiguration();
        exitMenu = true;
        break; //Salir y guardar
      case 8:
        readConfiguration();
        exitMenu = true;
        break; //Salir y cancelar cambios
      }
      forcePrint = true;
    }

    if (!exitMenu && (forcePrint || btnPressed != Button::Unknown))
    {
      forcePrint = false;

      static const byte endFor1 = (iMENU + rowsLCD - 1) / rowsLCD;
      int graphMenu = 0;

      for (int i = 1; i <= endFor1; i++)
      {
        if (idxMenu < i * rowsLCD)
        {
          graphMenu = (i - 1) * rowsLCD;
          break;
        }
      }

      byte endFor2 = graphMenu + rowsLCD;

      for (int i = graphMenu, j = 0; i < endFor2; i++, j++)
      {
        lcd.setCursor(1, j);
        lcd.print((i < iMENU) ? txtMENU[i] : "                ");
      }

      for (int i = 0; i < rowsLCD; i++)
      {
        lcd.setCursor(0, i);
        lcd.print(" ");
      }
      lcd.setCursor(0, idxMenu % rowsLCD);
      lcd.write(iARROW);
    }
  }

  lcd.clear();
}
void printDigits(byte digits)
{
  // utility function for digital clock display: prints preceding colon and leading 0
  if (digits < 10)
    lcd.print('0');
  lcd.print(digits, DEC);
}
/**
   MUESTRA EL SUBMENU EN EL LCD.

   @param menuID    ID del menu principal para usarlo como titulo del submenu
   @param screen    Segun el tipo, se representara el submenu de una forma u otra.
   @param value     Puntero a la variable que almacena el dato, y que se modificara.
   @param minValue  Valor minimo que puede tener la variable.
   @param maxValue  Valor maximo que puede tener la variable.
*/
void openSubMenu(byte menuID, Screen screen, int *value, int minValue, int maxValue)
{
  boolean exitSubMenu = false;
  boolean forcePrint = true;

  lcd.clear();

  while (!exitSubMenu)
  {
    btnPressed = readButtons();

    if (btnPressed == Button::Ok)
    {
      exitSubMenu = true;
    }
    else if (btnPressed == Button::Up && (*value) - 1 >= minValue)
    {
      (*value)--;
    }
    else if (btnPressed == Button::Down && (*value) + 1 <= maxValue)
    {
      (*value)++;
    }

    if (!exitSubMenu && (forcePrint || btnPressed != Button::Unknown))
    {
      forcePrint = false;

      lcd.setCursor(0, 0);
      lcd.print(txtMENU[menuID]);

      lcd.setCursor(0, 1);
      lcd.print("<");
      lcd.setCursor(columnsLCD - 1, 1);
      lcd.print(">");

      if (screen == Screen::Menu1)
      {
        lcd.setCursor(1, 1);
        lcd.print(txtSMENU1[*value]);
      }
      else if (screen == Screen::Menu2)
      {
        lcd.setCursor(1, 1);
        lcd.print(txtSMENU2[*value]);
      }
      else if (screen == Screen::Flag)
      {
        lcd.setCursor(columnsLCD / 2 - 1, 1);
        lcd.print(*value == 0 ? "NO" : "SI");
      }
      else if (screen == Screen::Number)
      {
        lcd.setCursor(columnsLCD / 2 - 1, 1);
        lcd.print(*value);
        lcd.print(" ");
      }
    }
  }

  lcd.clear();
}
void lcdSetup()
{
  // Inicia el LCD:
  lcd.begin(columnsLCD, rowsLCD);
  lcd.createChar(iARROW, bARROW);

  // Imprime la informacion del proyecto:
  lcd.setCursor(0, 0);
  lcd.print("  SkyConqueror  ");
  lcd.setCursor(0, 1);
  lcd.print("  Ver.  2021/06   ");
  delay(2000);
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("    Cargando ");
  lcd.setCursor(0, 1);
  for (int i = 0; i < columnsLCD; i++)
  {
    lcd.print(".");
    delay(150);
  }
  lcd.clear();
}
void miBTConfiguration()
{
  //SETUP BLUETOOTH (comentar en produccion)
  miBT.print("AT"); // Inicializa comando AT
  delay(1000);

  miBT.print("AT+NAME"); // Configura el nuevo nombre
  miBT.print(NOMBRE);
  delay(1000); // espera 1 segundo

  miBT.print("AT+BAUD"); // Configura la nueva velocidad
  miBT.print(BPS);
  delay(1000);

  miBT.print("AT+PIN"); // Configura el nuevo PIN
  miBT.print(PASS);
  delay(1000);
}
void resetPosition(){
  memory.d.target_x = 0;
  memory.d.target_y = 0;
}
/**
    LEE (Y CONFIGURA LA PRIMERA VEZ) LA MEMORIA EEPROM CON LA CONFIGURACION DE USUARIO
*/
void readConfiguration()
{
  for (int i = 0; i < sizeof(memory.d); i++)
    memory.b[i] = EEPROM.read(i);

  if (memory.d.initialized != 'Y')
  {
    memory.d.initialized = 'Y';
    memory.d.mode = 1;
    memory.d.target_x = 0;
    memory.d.target_y = 0;
    memory.d.time_show = 1;
    memory.d.time_on = 7;
    memory.d.time_off = 19;
    memory.d.time_unit = 0;

    writeConfiguration();
  }
}

/**
    ESCRIBE LA MEMORIA EEPROM CON AL CONFIGURACION DE USUARIO
*/
void writeConfiguration()
{
  for (int i = 0; i < sizeof(memory.d); i++){
    EEPROM.write(i, memory.b[i]);
    Serial.write(i+'-'+memory.b[i]);
  }
}

Button readBluetoothButtons()
{
  while (Serial.available() > 0)
  {
    command = Serial.read();
    Serial.print("Serial TX-");
    Serial.println(command);
  }
/*
  if (Serial.available())
  {
    Serial.println("Serial AVAILABLE");
    command = Serial.read();
  }

  if (miBT.available() > 0)
  {
    command = miBT.read();
    Serial.write(command);
  }

  if (miBT.available())
  {
    Serial.println("AVAILABLE");
    command = miBT.read();
  }
*/
  if (command == 'A')
  {
    memory.d.mode = 0;
    Serial.println("MODO PANEL SOLAR");
    
  }
  else if (command == 'a')
  {
    //digitalWrite(buzzer, LOW); // buzzer off
    
  }
  else if (command == 'b')
  {
    memory.d.mode = 1;
    Serial.println("MODO HELIOSTATO");
  }
  else if (command == 'c')
  { // turn on/off led's green
    memory.d.mode = 2;
    Serial.println("MODO MANUAL");

  }
  else if (command == 'd')
  { // turn on/off led's blue
    Serial.println("d");
  }
  else if (command == 'E')
  { // change led's stat to blink or off
    if (ledRojo == 1)
      ledRojo = 0;
    else
      ledRojo = 1;
    Serial.println("E");
  }
  else if (command == 'e')
  { // change led's stat to blink or off
    Serial.println("e");
  }
  else if (command == '8' && (memory.d.target_y + 1) <= 180)
  { // UP
    memory.d.target_y++;
    Serial.println("UP");
    Serial.println(memory.d.target_y);
  }
  else if (command == '2' && (memory.d.target_y - 1) >= 20)
  { // DOWN
     memory.d.target_y--;
    Serial.println("DOWN");
    Serial.println(memory.d.target_y);
  }
  else if (command == '4' && (memory.d.target_x - 1) >= 0)
  { // Left
    memory.d.target_x--;
    Serial.println("Left");
  }
  else if (command == '6' && (memory.d.target_x + 1) <= 359)
  { // Right
    memory.d.target_x++;
    Serial.println("Right");
  }
  else if (command == '7')
  { // forward & left

  
    Serial.println("Forward & Left");
  }
  else if (command == '9')
  { // forward & right
    Serial.println("Forward & Right");
  }
  else if (command == '1')
  { // backward & left
    Serial.println("Backward & Left");
  }
  else if (command == '3')
  { // backward & right
    Serial.println("Backward & Right");
  }
  else if (command == '5')
  { // stop
    Serial.println("Stop");
  }

  writeConfiguration();
}

/**
    LEE LOS DISTINTOS BOTONES DISPONIBLES Y DEVUELVE EL QUE HAYA SIDO PULSADO
        Este bloque de codigo varia dependiendo del tipo de teclado conectado.
*/
Button readButtons()
{
  currentButtonStateLeft = digitalRead(leftButtonPin);
  currentButtonStateRight = digitalRead(rightButtonPin);
  currentButtonStateUp = digitalRead(upButtonPin);
  currentButtonStateDown = digitalRead(downButtonPin);
  currentButtonStateEnter = digitalRead(enterButtonPin);
  currentButtonStateClear = digitalRead(clearButtonPin);

  // The current state for each button is set not equal to the pressed state and when it changes, the pressed state becomes equal to the current state.
  btnPressed = Button::Unknown;

  while (digitalRead(leftButtonPin) == HIGH)
  {
    btnPressed = Button::Left;
  }
  while (digitalRead(rightButtonPin) == HIGH)
  {
    btnPressed = Button::Right;
  }
  while (digitalRead(upButtonPin) == HIGH)
  {
    btnPressed = Button::Up;
  }
  while (digitalRead(downButtonPin) == HIGH)
  {
    btnPressed = Button::Down;
  }
  while (digitalRead(enterButtonPin) == HIGH)
  {
    btnPressed = Button::Ok;
  }

  while (btnPressed != Button::Unknown)
  {
    lcd.clear();
    return btnPressed;
  }

  /*
    Ver de comprar el lcd con botonera
    int val = analogRead(pPAD);
    btnPressed = Button::Unknown;

    if (val < 50)
    btnPressed = Button::Right;
    else if (val < 250)
    btnPressed = Button::Up;
    else if (val < 450)
    btnPressed = Button::Down;
    else if (val < 650)
    btnPressed = Button::Left;
    else if (val < 850)
    btnPressed = Button::Ok;
    //while (btnPressed != Button::Unknown && analogRead(pPAD) < 1000)
    return btnPressed;
  */
}

void giro(int paso_, int dire_, int habi_, int steps_)
{
  // digitalWrite(habi_, LOW);  // Habilita el Driver
  if (steps_ < 0)
  {
    digitalWrite(dire_, LOW);
    Serial.println("EN REVERSA");
    steps_ = steps_ * -1;
  }
  else
  {
    digitalWrite(dire_, HIGH);
    Serial.println("PARA ADELANTE");
  }
  for (int i = 0; i < steps_; i++)
  {
    // da  pasos por un tiempo
    digitalWrite(paso_, HIGH);
    delayMicroseconds(retardo);
    digitalWrite(paso_, LOW);
    delayMicroseconds(retardo);
  }
}

void show(char nameStr[], double val, boolean newline)
{
  Serial.print(nameStr);
  Serial.print("=");
  if (newline)
    Serial.println(val);
  else
    Serial.println(val);
}

void showTime(DateTime fecha)
{
  Serial.print(fecha.day());
  Serial.print("/");
  Serial.print(fecha.month());
  Serial.print("/");
  Serial.print(fecha.year());
  Serial.print(" ");
  Serial.print(fecha.hour());
  Serial.print(":");
  Serial.print(fecha.minute());
  Serial.print(":");
  Serial.println(fecha.second());
}

// MAIN LOOP
void setupMenu()
{

  // The program at this point is waiting for a button press.
  currentButtonStateLeft = digitalRead(leftButtonPin);
  currentButtonStateRight = digitalRead(rightButtonPin);
  currentButtonStateUp = digitalRead(upButtonPin);
  currentButtonStateDown = digitalRead(downButtonPin);
  currentButtonStateEnter = digitalRead(enterButtonPin);
  currentButtonStateClear = digitalRead(clearButtonPin);

  if (currentButtonStateLeft != lastButtonStateLeft || currentButtonStateRight != lastButtonStateRight ||
      currentButtonStateUp != lastButtonStateUp || currentButtonStateDown != lastButtonStateDown || currentButtonStateEnter != lastButtonStateEnter)
  // If there is a button push on any of the buttons, the following routine runs to check if it was a valid press:
  {
    lastDebounceTime = millis(); // lastDebounceTime is set equal to the running millis() function.
  }

  if ((millis() - lastDebounceTime) > debounceDelay)
  // If the lastDebounceTime (aka. the "snapshot" time) minus the running millis() function is higher than the set debounce delay, the following routine
  // below runs and checks which button was pushed:
  {

    // The current state for each button is set not equal to the pressed state and when it changes, the pressed state becomes equal to the current state.

    // LEFT BUTTON PRESS
    if (currentButtonStateLeft != buttonStateLeft) // Left button scrolls the menu options to the left.
    {
      buttonStateLeft = currentButtonStateLeft;

      if (buttonStateLeft == LOW) // Once the button is released, the push is registered and the code below runs.
      {
        directionPush--; // Both the up and down press counts will be reset to zero when the left button is pushed.
        upPressCount = 0;
        downPressCount = 0;
      }

      if (directionPush < 0) // If the user tries to scroll below the first menu option,
      {                      // the program will loop back to the last menu option.
        directionPush = 4;
      }
      lcd.clear();
    }

    // RIGHT BUTTON PRESS
    if (currentButtonStateRight != buttonStateRight) // Right button scrolls the menu options to the right.
    {
      buttonStateRight = currentButtonStateRight;

      if (buttonStateRight == LOW)
      {
        directionPush++; // Both the up and down press counts will be reset to zero when the right button is pushed.
        upPressCount = 0;
        downPressCount = 0;
      }

      if (directionPush > 4) // If the user tries to scroll above the last menu option,
      {                      // the program will loop back to the first menu option.
        directionPush = 0;
      }
      lcd.clear();
    }

    // UP BUTTON PRESS
    if (currentButtonStateUp != buttonStateUp) // Up button scrolls the setting upward.
    {
      buttonStateUp = currentButtonStateUp;

      if (buttonStateUp == LOW && directionPush == 0) // The first 5 times in which the "up" button is pushed, each push will add 1 increment to the setting.
      {
        upPressCount++;
        downPressCount = 0; // The downPressCount is reset to zero.
        setting1Counter++;

        if (upPressCount > 5) // If the "up" button is pushed more than 5 times consecutively, the setting increment increases by 5
        {                     // with every "up" button push and resets back when the down, left or right button is pushed.
          setting1Counter = setting1Counter + 4;
        }

        if (setting1Counter > 999) // Sets the setting counter limit to 999. The user cannot increase the counter beyond 999.
        {
          setting1Counter = 999;
        }
      }

      if (buttonStateUp == LOW && directionPush == 1)
      {
        upPressCount++;
        downPressCount = 0;
        setting2Counter++;

        if (upPressCount > 5)
        {
          setting2Counter = setting2Counter + 4;
        }

        if (setting2Counter > 999) // Sets the setting counter limit to 999. The user cannot increase the counter beyond 999.
        {
          setting2Counter = 999;
        }
      }

      if (buttonStateUp == LOW && directionPush == 2)
      {
        upPressCount++;
        downPressCount = 0;
        setting3Counter++;

        if (upPressCount > 5)
        {
          setting3Counter = setting3Counter + 4;
        }

        if (setting3Counter > 999) // Sets the setting counter limit to 999. The user cannot increase the counter beyond 999.
        {
          setting3Counter = 999;
        }
      }

      if (buttonStateUp == LOW && directionPush == 3)
      {
        upPressCount++;
        downPressCount = 0;
        setting4Counter++;

        if (upPressCount > 5)
        {
          setting4Counter = setting4Counter + 4;
        }

        if (setting4Counter > 999) // Sets the setting counter limit to 999. The user cannot increase the counter beyond 999.
        {
          setting4Counter = 999;
        }
      }

      if (buttonStateUp == LOW && directionPush == 4)
      {
        upPressCount++;
        downPressCount = 0;
        setting5Counter++;

        if (upPressCount > 5)
        {
          setting5Counter = setting5Counter + 4;
        }

        if (setting5Counter > 999) // Sets the setting counter limit to 999. The user cannot increase the counter beyond 999.
        {
          setting5Counter = 999;
        }
      }
      lcd.clear();
    }

    // DOWN BUTTON PRESS
    if (currentButtonStateDown != buttonStateDown) // Down button scrolls the setting downward.
    {
      buttonStateDown = currentButtonStateDown;

      if (buttonStateDown == LOW && directionPush == 0) // The first 5 times in which the "down" button is pushed, each push will subtract 1 increment to the setting.
      {
        downPressCount++;
        upPressCount = 0; // The upPressCount is reset to zero.
        setting1Counter--;

        if (downPressCount > 5) // If the "down" button is pushed more than 5 times consecutively, the setting increment decreases by 5
        {                       // with every "down" button push and resets back when the up, left or right button is pushed.
          setting1Counter = setting1Counter - 4;
        }

        if (setting1Counter < -999) // Sets the setting counter limit to -999. The user cannot increase the counter beyond -999.
        {
          setting1Counter = -999;
        }
      }

      if (buttonStateDown == LOW && directionPush == 1)
      {
        downPressCount++;
        upPressCount = 0;
        setting2Counter--;

        if (downPressCount > 5)
        {
          setting2Counter = setting2Counter - 4;
        }

        if (setting2Counter < -999) // Sets the setting counter limit to -999. The user cannot decrease the counter beyond -999.
        {
          setting2Counter = -999;
        }
      }

      if (buttonStateDown == LOW && directionPush == 2)
      {
        downPressCount++;
        upPressCount = 0;
        setting3Counter--;

        if (downPressCount > 5)
        {
          setting3Counter = setting3Counter - 4;
        }

        if (setting3Counter < 0) // This code prevents the user from entering
        {                        // a number below "0".
          setting3Counter = 0;   // Remove this code if you want to allow in
        }                        // negative numbers on a setting.
      }

      if (buttonStateDown == LOW && directionPush == 3)
      {
        downPressCount++;
        upPressCount = 0;
        setting4Counter--;

        if (downPressCount > 5)
        {
          setting4Counter = setting4Counter - 4;
        }

        if (setting4Counter < 0) // This code prevents the user from entering
        {                        // a number below "0".
          setting4Counter = 0;   // Remove this code if you want to allow in
        }                        // negative numbers on a setting.
      }

      if (buttonStateDown == LOW && directionPush == 4)
      {
        downPressCount++;
        upPressCount = 0;
        setting5Counter--;

        if (downPressCount > 5)
        {
          setting5Counter = setting5Counter - 4;
        }

        if (setting5Counter < 0) // This code prevents the user from entering
        {                        // a number below "0".
          setting5Counter = 0;   // Remove this code if you want to allow in
        }                        // negative numbers on a setting.
      }

      lcd.clear();
    }
  }

  // ENTER BUTTON PRESS
  if (currentButtonStateEnter != buttonStateEnter)
  {
    buttonStateEnter = currentButtonStateEnter;

    if (buttonStateEnter == LOW && directionPush == 0) // The Enter button simply enters the setting and flashes a brief message.
    {                                                  // Please feel free to expand on this code to add more functions.
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("DEGREES FAHREN.");
      lcd.setCursor(0, 1);
      lcd.print("IS ENTERED");
      delay(2000);
    }

    if (buttonStateEnter == LOW && directionPush == 1)
    {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("DEGREES CELSIUS");
      lcd.setCursor(0, 1);
      lcd.print("IS ENTERED");
      delay(2000);
    }

    if (buttonStateEnter == LOW && directionPush == 2)
    {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("WEIGHT IN LBS.");
      lcd.setCursor(0, 1);
      lcd.print("IS ENTERED");
      delay(2000);
    }

    if (buttonStateEnter == LOW && directionPush == 3)
    {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("INCHES");
      lcd.setCursor(0, 1);
      lcd.print("IS ENTERED");
      delay(2000);
    }

    if (buttonStateEnter == LOW && directionPush == 4)
    {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("MILES PER HOUR");
      lcd.setCursor(0, 1);
      lcd.print("IS ENTERED");
      delay(2000);
    }
    lcd.clear();
  }

  // CLEAR BUTTON PRESS
  if (currentButtonStateClear != buttonStateClear)
  {
    buttonStateClear = currentButtonStateClear;

    if (buttonStateClear == LOW && directionPush == 0) // The Clear button clears all setting data depending on what menu option you are viewing.
    {                                                  // It flahses a brief message stating that the data has been cleared.
      lcd.clear();                                     // The press counts for both the up and down variables are also reset to zero.
      lcd.setCursor(0, 0);
      lcd.print("DEGREES FAHREN.");
      lcd.setCursor(0, 1);
      lcd.print("IS CLEARED");
      setting1Counter = 0;
      downPressCount = 0;
      upPressCount = 0;
      delay(2000);
    }

    if (buttonStateClear == LOW && directionPush == 1)
    {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("DEGREES CELSIUS");
      lcd.setCursor(0, 1);
      lcd.print("IS CLEARED");
      setting2Counter = 0;
      downPressCount = 0;
      upPressCount = 0;
      delay(2000);
    }

    if (buttonStateClear == LOW && directionPush == 2)
    {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("WEIGHT IN LBS.");
      lcd.setCursor(0, 1);
      lcd.print("IS CLEARED");
      setting3Counter = 0;
      downPressCount = 0;
      upPressCount = 0;
      delay(2000);
    }

    if (buttonStateClear == LOW && directionPush == 3)
    {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("INCHES");
      lcd.setCursor(0, 1);
      lcd.print("IS CLEARED");
      setting4Counter = 0;
      downPressCount = 0;
      upPressCount = 0;
      delay(2000);
    }

    if (buttonStateClear == LOW && directionPush == 4)
    {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("MILES PER HOUR");
      lcd.setCursor(0, 1);
      lcd.print("IS CLEARED");
      setting5Counter = 0;
      downPressCount = 0;
      upPressCount = 0;
      delay(2000);
    }
    lcd.clear();
  }

  // After a button is pushed and the count recorded, all the states reset back to LOW for the data to be processed correctly.
  lastButtonStateLeft = currentButtonStateLeft;   // resets the left button state to LOW
  lastButtonStateRight = currentButtonStateRight; // resets the right button state to LOW
  lastButtonStateUp = currentButtonStateUp;       // resets the up button state to LOW
  lastButtonStateDown = currentButtonStateDown;   // resets the down button state to LOW
  lastButtonStateEnter = currentButtonStateEnter; // resets the enter button state to LOW
  lastButtonStateClear = currentButtonStateClear; // resets the clear button state to LOW
}
////////////////////////////////////////////
///////     Botonera
////////////////////////////////////////////

//////////////////////////////////////////////////
//This code calculates the angles for the heliostat
//(returnaltaz = 1 will return alt, 2 returns az)
//////////////////////////////////////////////////
float FindHeliostatAngle(float altitude, float azimuth, float targetalt, float targetaz, int returnAltAz)
{
  float x, y, z, z1, z2, x1, x2, y1, y2, hyp, dist, machinealt, machineaz;

  //////////////////////////////////////////////////
  //The cartisian location of the sun with respect to Helios
  //////////////////////////////////////////////////
  z1 = cos(to_rad(offset_Elv - altitude));     //The altitude with respect to helios
  hyp = sin(to_rad(offset_Elv - altitude));    //The altitude with respect to helios
  x1 = hyp * cos(to_rad(offset_Az - azimuth)); //The azimuth with respect to helios
  y1 = hyp * sin(to_rad(offset_Az - azimuth)); //The azimuth with respect to helios

  //////////////////////////////////////////////////
  //The cartisian location of the target with respect to Helios
  //////////////////////////////////////////////////
  z2 = cos(to_rad(targetalt));      //The target with respect to helios
  hyp = sin(to_rad(targetalt));     //The target with respect to helios
  x2 = hyp * cos(to_rad(targetaz)); //The target with respect to helios
  y2 = hyp * sin(to_rad(targetaz)); //The target with respect to helios

  //////////////////////////////////////////////////
  //The cartisian definition for the vector pointing to along
  //the bisector of the sun's location and the target's location
  //////////////////////////////////////////////////
  x = (x1 + x2);
  y = (y1 + y2);
  z = (z1 + z2);
  dist = sqrt(x * x + y * y + z * z);

  //////////////////////////////////////////////////
  //The spherical definition of the vector pointing along
  //the bisector of the sun's location and the target's location
  //////////////////////////////////////////////////
  if ((dist > -0.00000001) && (dist < 0.00000001))
  {
    Serial.println('Es imposible reflejar al objetivo con este angulo');
  }
  else
  {
    machinealt = to_deg(acos(z / dist)); //Angle below the vertical, how this servo is controlled.
  }

  if (x == 0)
  {
    x = 0.000001;
  }
  machineaz = to_deg(atan2(y, x));
  if (returnAltAz == 1)
  {
    return machinealt;
  }
  if (returnAltAz == 2)
  {
    return machineaz;
  }
}

float to_rad(float angle)
{
  return angle * (pi / 180);
}

float to_deg(float angle)
{
  return angle * (180 / pi);
}

void scanBaudrate()
{
  unsigned long bauds[12] = {300, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 74880, 115200, 230400, 250000};
  for (int i = 0; i < 12; i++)
  {
    miBT.begin(bauds[i]);
    delay(10);
    miBT.print(F("Baudrate "));
    miBT.println(bauds[i]);
    commandBau("AT", 2000);
  }
}

String commandBau(const char *toSend, unsigned long milliseconds)
{
  String result;
  miBT.print("Sending: ");
  miBT.println(toSend);
  miBT.print(toSend);
  unsigned long startTime = millis();
  miBT.print(F("Received: "));
  while (millis() - startTime < milliseconds)
  {
    if (miBT.available())
    {
      char c = miBT.read();
      miBT.write(c);
      result += c; // append to the result string
    }
  }
  miBT.println(); // new line after timeout.
  return result;
}
