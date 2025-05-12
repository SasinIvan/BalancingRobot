
//=======================================================================================//
//  MPU6050
//=======================================================================================//
// библиотека I2Cdevlib для взаимодействия с гироскопом/акселерометром MPU6050 через шину I2C 
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"

// библиотека ардуины для работы с I2C шиной
#include "Wire.h"
#include <SoftwareSerial.h> //Bluetooth
SoftwareSerial BTSerial(3, 8);
// Удобная бибилотека для работы с EEPROM памятью микроконтроллера
// EEPROM - энергонезависимая память, в неё сохраняются значения калибровки MPU6050
#include "EEPROMex.h"
#define CALIBRATION_FLAG_PIN 2 //пин флага калибровки, если на нём низкий уровень во время инициализации(джампер замкнут) - 
                               //то будет произведена калибровка и сохранение значений, иначе просто загрузка значений из eeprom в mpu6050
#define START_BYTE 1010 //начальный адрес eeprom куда будет идти запись

// переменные нужные для работы с mpu6050
bool dmpInitialized = false;  // будет true если инициализация DMP процессора прошла удачно
uint8_t devStatus;      // еще один флаг, если 0 то mpu6050 проинициализирован
uint8_t fifoBuffer[64]; // буффер в который приходят данные от mpu6050

// переменные для расчета ориентации в пространстве
Quaternion q;           // [w, x, y, z]         кватернион, это то что выдает DMP процессор mpu6050
VectorFloat gravity;    // [x, y, z]            вектор гравицации
float ypr[3];           // [yaw, pitch, roll]   крен, тангаж, рысканье в радианах

// экземпляр класса MPU6050
MPU6050 mpu;

// инициализация и калибровка mpu6050
void initializeMpu() {
  mpu.reset(); //рестарт датчика на всякий случай
  delay(100);
  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  
  if (devStatus == 0) {
    int offsets[6]; // для значений калибровки
    
    if (digitalRead(CALIBRATION_FLAG_PIN)) { //если джампер не установлен

      Serial.println("Reading offsets from EEPROM");
      
      // чтение значений из eeprom
      for (byte i = 0; i < 6; i++) {
          offsets[i] = EEPROM.readInt(START_BYTE + i * 2); //i * 2 т.к. тип данных int занимает 2 байта
      }
    
      // установка значений калибровки в mpu6050
      mpu.setXAccelOffset(offsets[0]); 
      mpu.setYAccelOffset(offsets[1]); 
      mpu.setZAccelOffset(offsets[2]); 
      mpu.setXGyroOffset(offsets[3]); 
      mpu.setYGyroOffset(offsets[4]); 
      mpu.setZGyroOffset(offsets[5]); 
      
    } else { // если джампер установлен

      Serial.println("Calibrating offsets");
      
      // 15 циклов калибровки
      mpu.CalibrateAccel(15);
      mpu.CalibrateGyro(15);

      // считываются получившиеся значения после калибровки
      offsets[0] = mpu.getXAccelOffset();
      offsets[1] = mpu.getYAccelOffset();
      offsets[2] = mpu.getZAccelOffset();
      offsets[3] = mpu.getXGyroOffset();
      offsets[4] = mpu.getYGyroOffset();
      offsets[5] = mpu.getZGyroOffset();

      Serial.println("Saving offsets to EEPROM");

      // запись в память eeprom
      for (byte i = 0; i < 6; i++) {
          EEPROM.updateInt(START_BYTE + i * 2, offsets[i]);
      }      
    }

    // debug
    mpu.PrintActiveOffsets();
     
    mpu.setDMPEnabled(true);
    Serial.println("DMP good");
    dmpInitialized = true;
    
  } else {
    Serial.println("DMP Initialization failed");
  }
  
}

//=======================================================================================//
//  УПРАВЛЕНИЕ ШАГОВЫМИ ДВИГАТЕЛЯМИ
//=================================================== ====================================//
// эти дефайны ниже - это пины порта Д самого микроконтроллера на плате ардуины atmega328p, а не номера пинов ардуины!
// но для порта Д и то и то совпадает...
#define STEP_PIN 6
#define DIR_PIN 5

// какие уровни на выводе dir нужны для направления вперед и назад
#define FORWARD_DIR 1 
#define BACKWARD_DIR 0

#define ENABLE_PIN 7 // пин включения/выключения двигателей

#define TICKS_PER_SECOND 32000 // количество вызовов прерывания таймера в секунду
#define VIRTUAL_TICKS_PER_STEP 2147483647 // = (2^32)/2, "виртуальное" число, когда counter достигает этого значения - тогда пора делать шаг, 
                                          //нужно чтобы обойтись без переменных с плавающей точкой

int16_t speed = 0; // текущая скорость вращения моторов [шаг/сек]
char ch; 
int const_speed = 0;//постоянная скорость двигателя
// структура данных содержащая в себе данные необходимые для принятия решения - шагать или не шагать
typedef struct {
  uint8_t dir; // направление вращения
      
  uint32_t counter; //счетчик "виртуальных" тиков
  uint32_t negative_counter; //еще счетчик нужен для обработки ситуации смены направления движения
  uint32_t step_rate; //количество "виртуальных" тиков прибавляемых к счетчику каждое прерывание таймера, с 0 по (2^32)/2
} motor_values;

volatile motor_values current_values; // текущие данные
volatile motor_values new_values; // новые данные, становятся текущими в прерывании чтобы не получилось накладок
volatile bool new_values_ready = false; // флаг что пора обновить current_values

// маски для установки уровней step и dir напрямую в порт микроконтроллера, ардуиновские функции очень медленные
volatile byte port_d_step_mask = 0b00000000;
volatile byte port_d_dir_mask = 0b00000000;


// этот обработчик прерывания управляет драйвером шагового мотора формируя step импульсы и уровень на dir выводе
ISR(TIMER1_COMPA_vect)
{
  //debug
  //PORTC |= 0b00000010; used to see processing time usage by using logic analyzer
  //

  //////формируется STEP импульс если нужно
  //вычисление port_d_step_mask не стабильно по времени, поэтому импульсы формируются в начале следующего выполнения обработчика, 
  //так получается точнее контролировать скорость вращения двигателей

  if (port_d_step_mask != 0b00000000) {

    PORTD |= port_d_step_mask; // быстро делаем 1 на пине где step 

    //ждем 0.5 микросекунды выполняя 8 пустых инструкций микроконтроллера
    __asm__ __volatile__ (
    "nop" "\n\t"    "nop" "\n\t"    "nop" "\n\t"    "nop" "\n\t"
    "nop" "\n\t"    "nop" "\n\t"    "nop" "\n\t"    "nop");
    
    PORTD &= ~(port_d_step_mask); // и обратно сбрасываем в 0
    port_d_step_mask = 0b00000000;
  }
  //

  //////расчёт port_d_step_mask, установка/сброс dir пинов 
  
  //
  if (new_values_ready) {
    
    //смена направления
    if (current_values.dir != new_values.dir) {
      current_values.dir = new_values.dir;
      current_values.negative_counter = current_values.counter;
      current_values.counter = 0;

      //установка или сброс dir пина
      PORTD &= ~(1 << DIR_PIN); //reset
      PORTD |= current_values.dir << DIR_PIN;
    }

    current_values.step_rate = new_values.step_rate; // устанавливаем новый step_rate
    new_values_ready = false;
  }

  // подсчет виртуальных тиков
  if (current_values.negative_counter == 0) {
    current_values.counter += current_values.step_rate;
    
  } else { // когда изменилось направление

    if (current_values.negative_counter > current_values.step_rate) {
      current_values.negative_counter -= current_values.step_rate;
    } else {
      current_values.counter += current_values.step_rate - current_values.negative_counter;
      current_values.negative_counter = 0;
    }
    
  }

  // смотрим делать шаг или пока не надо
  if (current_values.counter >= VIRTUAL_TICKS_PER_STEP) {
    current_values.counter -= VIRTUAL_TICKS_PER_STEP;
    port_d_step_mask |= (1 << STEP_PIN);
  }
    
  //debug
  //PORTC &= ~(0b00000010); used to see processing time usage by using logic analyzer
  //
}

// функция установки скорости вращения двигателей
void setSpeed(int16_t speed) {
  new_values.dir = speed > 0 ? FORWARD_DIR : BACKWARD_DIR;
  
  new_values.step_rate = (VIRTUAL_TICKS_PER_STEP/TICKS_PER_SECOND) * abs(speed);
  new_values_ready = true;
}

//=======================================================================================//
//  ПИД регуляторы
//=======================================================================================//
#include "PID.h"

//переменные для вычисления дельты времени
uint32_t previous_micros;
uint32_t current_micros;
float dt;

//целевые значения
float target_speed = 0;
float target_angle = 0;

// коэффициенты ПИДов, определяют поведение робота
#define SPEED_KP 0.000015 // позволяет достичь целевой скорости
//#define SPEED_KI 0.0000008
#define SPEED_KI 0.0000000  // когда > 0, у робота появляется стремление сократить пройденный путь, посколько интеграл от скорости - путь
#define SPEED_KD 0.00000027 // позволяет с меньшими колебаниями достичь целевой скорости

#define ANGLE_KP 2100.0 // позволяет достичь целевого угла
//#define ANGLE_KI 8500.0
#define ANGLE_KI 300.0 // убирает статическую ошибку отклонения угла от целевого
#define ANGLE_KD 180.0 // позволяет с меньшими колебаниями достичь заданного угла

// максимумы
#define MAX_SPEED 15000 // шаги в секунду
#define MAX_ACCEL 700.0 // шаги в секунду в квадрате

#define FORWARD_BACKWARD_SPEED 100 // шаги в секунду
#define LEFT_RIGHT_SPEED 50 // шаги в секунду

//PID    name(  KP    ,   KI    ,   KD    , min val,    max val,    min integral, max integral);
PID speed_pid(SPEED_KP, SPEED_KI, SPEED_KD, -M_PI/3,    M_PI/3,     0,            0); // вычисляет целевой угол наклона с целью достич целевую скорость
PID angle_pid(ANGLE_KP, ANGLE_KI, ANGLE_KD, -MAX_ACCEL, MAX_ACCEL,  -0.35,        0.35); // вычисляет целевое угловое ускорение валов моторов с целью достич целевой угол наклона


void setup() {
  //////для отладки
  Serial.begin(115200);
  BTSerial.begin(9600); // Инициализируем программное последовательное соединение для Bluetooth модуля на скорости 38400.
  //pinMode(A0, OUTPUT);//PC0
  //pinMode(A1, OUTPUT);//PC1
  //////

  //////пины драйвера
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, HIGH); // выключить моторы

  //defined pins are MCs port d pin numbers
  //but there are used arduino pin names, works because they are the same for port D
  // эти функции используют номера пинов АРДУИНО, а значения заданные в  STEP_PIN и DIR_PIN это пины порта Д микроконтроллера, 
  // но на этом порту они совпадают, поэтому и так сойдет
  pinMode(STEP_PIN, OUTPUT);
  digitalWrite(STEP_PIN, LOW);
  
  pinMode(DIR_PIN, OUTPUT); 
  //////
  
  //////НАЧАЛЬНЫЕ ЗНАЧЕНИЯ ДЛЯ МОТОРОВ
  current_values.dir = 2; // заставит прерывание обработать случай смены направления
  current_values.counter = 0;
  current_values.negative_counter = 0;
  current_values.step_rate = 0;
  //////

  //////инициализация I2C шины для общения с mpu6050
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock
  //////

  //////инициализация MPU6050
  pinMode(CALIBRATION_FLAG_PIN, INPUT_PULLUP);
  
  initializeMpu();
  digitalWrite(ENABLE_PIN, LOW); //enable motors
  //////

  //////инциализация таймера 1
  //CTC режим, no prescaling, подробней смотреть в даташит микроконтроллера
  TCCR1A = 0;
  TCCR1B = (1<<WGM12) | (1<<CS10); 
  
  OCR1A = 0x01F3; //0x01F3 = 499, 32000 Гц прерывания таймера 1
  TIMSK1 |= (1<<OCIE1A); //разрешить срабатывание прерывания 
  //////

  // для расчета дельты времени
  previous_micros = micros();
}

void loop() {
  if (!dmpInitialized) return;
    //debug
    //PORTC |= 0b00000001; //used to see processing time usage by using logic analyzer
    //
    while (!mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {}// пытается получить данные mpu6050, пока не получится

    mpu.dmpGetQuaternion(&q, fifoBuffer); //вычленяет кватернион из буффера
    mpu.dmpGetGravity(&gravity, &q); //расчитывает вектор гравитации на основе полученного кватерниона
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); //вычисление крена, тангажа и рысканья используя кватернион и вектор гравитации, TODO: убрать лишние расчеты

    /* debug
    Serial.print("y: ");
    Serial.print(ypr[0]);
    Serial.print("p: ");
    Serial.print(ypr[1]);
    Serial.print("r: ");
    Serial.print(ypr[2]);
    */
    
    //расчет дельты времени
    current_micros = micros();
    dt =  ((float)(current_micros - previous_micros))/1000000.0; //dt [сек]
    previous_micros = current_micros;  
  
    //вся магия ПИД регуляторов позволяющая роботу балансировать
    target_angle = speed_pid.update(target_speed, (float)speed, dt);
    speed += -angle_pid.update(target_angle, ypr[1], dt);

    if (BTSerial.available()) {
      ch = BTSerial.read();  // Читаем символ из Bluetooth модуля.
      if(ch == 'F')
        const_speed = 5000;  
      else if(ch == 'B')
        const_speed = -5000;
      else if(ch == '0')
        const_speed = 0;
    }
    //ограничение скорости
    if (speed + const_speed > MAX_SPEED) speed = MAX_SPEED;
    if (speed + const_speed < -MAX_SPEED) speed = -MAX_SPEED;
    // Ожидаем и выводим ответ от Bluetooth модуля.+
    setSpeed(speed + const_speed);
    Serial.println(speed);
    
  //debug
  //PORTC &= ~(0b00000001); //used to see processing time usage by using logic analyzer
  //
    
  
}
