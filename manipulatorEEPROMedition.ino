#include <AccelStepper.h>
#include "GyverEncoder.h"
#include <Servo.h>
#include <EEPROM.h>
#define MOVE_SPEED 100 // скорость движения моторов при калибровке
#define MARGINE_ERROR 2 // допустимая погрешность установки моторов
#define STEPS_TO_REMEMBER 8 // количество шагов для зароминания
#define STATE_TERMS 5
#define SB3 52

// Определение констант управления мотором

Servo myservo1;
Servo myservo2;

Encoder enc1(22, 24, 26);
int Z = 0;
Encoder enc2(28, 30, 32);
int Y = 0;
Encoder enc3(34, 36, 38);
int X = 0;
Encoder enc4(40, 42, 44);
int A = 0;
Encoder enc5(46, 48, 50);
int Q = 0;

// блок задания пинов
const int enablePin = 8;  // пин управления энергопотреблением
const int xdirPin = 5;     // пин управления направлением движения оси X
const int xstepPin = 2;    // пин управления шаговым двигателем оси X
const int ydirPin = 6;     // пин управления направлением движения оси Y
const int ystepPin = 3;    // пин управления шаговым двигателем оси Y
const int zdirPin = 7;     // пин управления направлением движения оси Z
const int zstepPin = 4;    // пин управления шаговым двигателем оси Z
const int xEndstopPin = 11;  // пин концевика оси X
const int yEndstopPin = 10; // пин концевика оси Y
const int zEndstopPin = 9; // пин концевика оси Z

AccelStepper stepper1(1,xstepPin,xdirPin);// создание объекта шагового мотора 1
AccelStepper stepper2(1,ystepPin,ydirPin);// создание объекта шагового мотора 2
AccelStepper stepper3(1,zstepPin,zdirPin);// создание объекта шагового мотора 3

// пустой массив для заполнения его будущими позициями для перемещения 
int actionQueue[STEPS_TO_REMEMBER][STATE_TERMS]; 

// переменная которая означает номер последней контрольной точки
int lastState = 0;

// флаг режима обучения: 1 - режим обучения включен; -1 - режим обучения выключен
int memorizationModeFlag = -1; 

// флаг захвата: 1 - захват; -1 - отпускание
int grabModeFlag = -1; 

// [0]xstepper [1]ystepper [2]zstepper [3]servoRotation [4]servoGrab

// функция запоминания позиции: заносит в массив actionQueue 5 координат всякий раз когда фукция вызывается, автоматически фиксирует
// последнюю контрольную точку lastState, не может быть вызвана если количество позиция для запоминания превышает STEPS_TO_REMEMBER

void partitionInt(int integer, byte* byteStorage) {

  byteStorage[0] = integer & 0x0F;
  byteStorage[1] = (integer & 0xF0) >> 4;
}

int combineIntBytes(byte* byteStorage) {
  return (int) (byteStorage[1] << 4) | byteStorage[0];
}


void sendToEEPROM() {
  Serial.println("Sending to EEPROM start."); 
  byte coordinateBytes[2];
  int k = 0;
  for(int i = 0; i < STEPS_TO_REMEMBER; ++i) {
    for(int j = 0; j < STATE_TERMS; ++j) {  
      partitionInt(actionQueue[i][j], coordinateBytes);
      EEPROM.update(k++, coordinateBytes[1]);
      EEPROM.update(k++, coordinateBytes[0]);
      Serial.println("Updating [" + (String) i + ']' + '[' + (String) j + "] = " + (String) actionQueue[i][j]);
    }
  }
  EEPROM.update(k, lastState);
  Serial.println("sanded");
} 
  void readFromEEPROM() {
    Serial.println("start reading"); 
    byte coordinateBytes[2];
    int k = 0;
    for(int i = 0; i < STEPS_TO_REMEMBER; ++i) {
     for(int j = 0; j < STATE_TERMS; ++j) {  
        coordinateBytes[1] = EEPROM.read(k++);
        coordinateBytes[0] = EEPROM.read(k++);
        int value = combineIntBytes(coordinateBytes);
        Serial.println("Read [" + (String) i + ']' + "[" + (String) j + "] = " + (String) value );
        actionQueue[i][j] = value;
        
      }
    }
    if(EEPROM.read(k) != 255)
      lastState = EEPROM.read(k);
    
   }
  
  
  

void rememberState(){
      if(lastState < STEPS_TO_REMEMBER) {
        int i = 0;
        actionQueue[lastState][i++] = stepper1.currentPosition();
        actionQueue[lastState][i++] = stepper2.currentPosition();
        actionQueue[lastState][i++] = stepper3.currentPosition();
        actionQueue[lastState][i++] = myservo2.read();
        actionQueue[lastState][i++] = myservo1.read();
        ++lastState;
        Serial.println("State remembered.");
      }
      if(lastState == STEPS_TO_REMEMBER)
        Serial.println("Cannot store more states.");
  }

// функция очищения массива actionQueue (заполняет его нулями)
void purgeQueue(){
  
  for(int i = 0; i < STEPS_TO_REMEMBER; ++i)
    for(int j = 0; j < STATE_TERMS; ++j)
      actionQueue[i][j] = 0;
  }

// фукция воспроизведения маршрута по точкам из режима обучения
void replayMemorized() {
   Serial.println("Begining replay...");
   int j = 0;
   int i = lastState;
   //lastState = 0;
   while(i > j){
     Serial.println("Replaying state " + (String) (j + 1) + '.');
     manipGoToPosition(actionQueue[j][0], actionQueue[j][1], actionQueue[j][2], actionQueue[j][3], actionQueue[j][4]);
     delay(1000);
     ++j;
   }
   //purgeQueue();
   Serial.println("Finished replaying");
}

// старая прога
/*void beginMemorization(){
  
  purgeQueue();
  Serial.println("Begining memorization...");
  while(lastState < STEPS_TO_REMEMBER || !enc2.isClick()) {
     if (enc1.isClick()) { 
      Serial.println("State " + (String) lastState + '.');
      int j = 0;
      actionQueue[lastState][j++] = stepper1.currentPosition();
      actionQueue[lastState][j++] = stepper2.currentPosition();
      actionQueue[lastState][j++] = stepper3.currentPosition();
      actionQueue[lastState][j++] = myservo2.read();
      actionQueue[lastState][j++] = myservo1.read();
      ++lastState;
      Serial.println("State " + (String) lastState + " is memorized.");     
     }
     
   }
}
*/

void setup() {
  
  pinMode(xstepPin,OUTPUT);     // пин управления шаговым двигателем оси X установлен в режим вывода
  pinMode(xdirPin,OUTPUT);      // пин управления направлением движения оси X установлен в режим вывода
  pinMode(ystepPin,OUTPUT);     // пин управления шаговым двигателем оси Y установлен в режим вывода
  pinMode(ydirPin,OUTPUT);      // пин управления направлением движения оси Y установлен в режим вывода
  pinMode(zstepPin,OUTPUT);     // пин управления шаговым двигателем оси Z установлен в режим вывода
  pinMode(zdirPin,OUTPUT);      // пин управления направлением движения оси Z установлен в режим вывода
  
  pinMode(enablePin,OUTPUT);   // пин управления энергопотреблением установлен в режим вывода
  digitalWrite(enablePin,LOW); // установка пина управления энергопотреблением на низкий уровень, чтобы включить драйвер шаг
  stepper1.setMaxSpeed(5000.0);     // установка максимальной скорости мотора 1 на 300 шагов в секунду
  stepper1.setAcceleration(1500.0);  // установка ускорения мотора 1 на 20 шагов в секунду в квадрате
  stepper2.setMaxSpeed(5000.0);     // установка максимальной скорости мотора 2 на 300 шагов в секунду
  stepper2.setAcceleration(1500.0);  // установка ускорения мотора 2 на 20 шагов в секунду в квадрате
  stepper3.setMaxSpeed(5000.0);     // установка максимальной скорости мотора 3 на 300 шагов в секунду
  stepper3.setAcceleration(1500.0);  // установка ускорения мотора 3 на 20 шагов в секунду в квадрате

  pinMode(xEndstopPin, INPUT_PULLUP);   // пин концевика оси X установлен в режим входа с подтяжкой к питанию
  pinMode(yEndstopPin, INPUT_PULLUP);   // пин концевика оси Y установлен в режим входа с подтяжкой к питанию
  pinMode(zEndstopPin, INPUT_PULLUP);   // пин концевика оси Z установлен в режим входа с подтяжкой к питанию

  Serial.begin(9600);
  enc1.setType(TYPE2);        // тип энкодера TYPE1 одношаговый, TYPE2 двухшаговый. Если ваш энкодер работает странно, смените тип\=
  enc1.setFastTimeout(40);    // таймаут на скорость isFastR. По умолч. 50

  enc2.setType(TYPE2);        // тип энкодера TYPE1 одношаговый, TYPE2 двухшаговый. Если ваш энкодер работает странно, смените тип\=
  enc2.setFastTimeout(40);    // таймаут на скорость isFastR. По умолч. 50

  enc3.setType(TYPE2);        // тип энкодера TYPE1 одношаговый, TYPE2 двухшаговый. Если ваш энкодер работает странно, смените тип\=
  enc3.setFastTimeout(40);    // таймаут на скорость isFastR. По умолч. 50

  enc4.setType(TYPE2);        // тип энкодера TYPE1 одношаговый, TYPE2 двухшаговый. Если ваш энкодер работает странно, смените тип\=
  enc4.setFastTimeout(40);    // таймаут на скорость isFastR. По умолч. 50

  enc5.setType(TYPE2);        // тип энкодера TYPE1 одношаговый, TYPE2 двухшаговый. Если ваш энкодер работает странно, смените тип\=
  enc5.setFastTimeout(40);    // таймаут на скорость isFastR. По умолч. 50

  myservo1.attach(A0);//захват
  myservo2.attach(A1);//поворот захвата

  myservo1.write(0);//поворот на градусы
  
  Serial.println("Starting zero setup...");
  manipZeroSetup(stepper1, stepper2, stepper3, xEndstopPin, yEndstopPin, zEndstopPin, MOVE_SPEED);
  Serial.println("Finished zero setup.");
  Serial.println("Preparing queue...");
  purgeQueue();
  readFromEEPROM();
  Serial.println("Finished preparing queue.");
  
}

void manipZeroSetup(AccelStepper stepperX, AccelStepper stepperY, AccelStepper stepperZ, int xEndstopPin, int yEndstopPin, int zEndstopPin, int movementSpeed)
{
     while(digitalRead(xEndstopPin) == LOW || digitalRead(yEndstopPin) == LOW || digitalRead(zEndstopPin) == LOW) {
        
          setMotorToZeroPoint(stepperX, xEndstopPin, -movementSpeed);
          setMotorToZeroPoint(stepperY, yEndstopPin, movementSpeed);
          setMotorToZeroPoint(stepperZ, zEndstopPin, -movementSpeed);
    }
}

void setMotorToZeroPoint(AccelStepper stepper, int endstopPin, int movementSpeed) {
  
   if (digitalRead(endstopPin) == HIGH) {  // если концевик оси нажат
    stepper.setCurrentPosition(0);         // установить текущую позицию мотора в ноль
    stepper.stop();                        // остановить движение мотора
  }
  else
    stepper.move(movementSpeed);

  stepper.run();
   
}

// функция перемещение к заданным координатам, принимает на вход координаты каждого из 3-х шаговиков
// и оба сервопривода, статическая ошибка MARGINE_ERROR, новые координаты перезаписываются в память автоматически
void manipGoToPosition(int x, int y, int z, int a, int q){
  while(abs(stepper1.currentPosition() - x) > MARGINE_ERROR || abs(stepper2.currentPosition() - y) > MARGINE_ERROR || abs(stepper3.currentPosition() - z) > MARGINE_ERROR)
  {
    stepper1.moveTo(x);
    stepper2.moveTo(y);
    stepper3.moveTo(z);
    stepper1.run();
    stepper2.run();
    stepper3.run();
  }
  myservo1.write(q);
  myservo2.write(a);
  X = stepper1.currentPosition(); //синхрониазция заданных координат с действительными
  Y = stepper2.currentPosition(); //синхрониазция заданных координат с действительными
  Z = stepper3.currentPosition(); //синхрониазция заданных координат с действительными
  A = myservo2.read();  //синхрониазция заданных координат с действительными
  Q = myservo1.read();  //синхрониазция заданных координат с действительными
 }

void loop() {

  // поворот первым шаговиком с помощью 1 енкодера
  // быстрое вращение и медленное вращение 
  // координаты автоматически фиксируются в памяти
  enc1.tick();
  if (enc1.isRight()) Z -= 10;        // если был поворот направо, увеличиваем на 1
  if (enc1.isLeft()) Z += 10;         // если был поворот налево, уменьшаем на 1
  if (enc1.isFastR()) Z -= 35;        // если был быстрый поворот направо, увеличиваем на 10
  if (enc1.isFastL()) Z += 35;        // если был быстрый поворот налево, уменьшаем на 10
  if (enc1.isTurn()) {                // если был совершён поворот (индикатор поворота в любую сторону)
    Serial.println(Z);                // выводим значение при повороте
  }
  stepper3.moveTo(Z); 
  stepper3.run();   

  // поворот вторым шаговиком с помощью 2 енкодера
  // быстрое вращение и медленное вращение 
  // координаты автоматически фиксируются в памяти
  enc2.tick();
  if (enc2.isRight()) Y += 10;        
  if (enc2.isLeft()) Y -= 10;         
  if (enc2.isFastR()) Y += 35;        
  if (enc2.isFastL()) Y -= 35;        
  if (enc2.isTurn()) {                
    Serial.println(Y);                
  } 
  stepper2.moveTo(Y); 
  stepper2.run();  

  // поворот третим шаговиком с помощью 3 енкодера
  // быстрое вращение и медленное вращение 
  // координаты автоматически фиксируются в памяти
  enc3.tick();
  if (enc3.isRight()) X -= 10;       
  if (enc3.isLeft()) X += 10;        
  if (enc3.isFastR()) X -= 35;    
  if (enc3.isFastL()) X += 35;    
  if (enc3.isTurn()) {            
    Serial.println(X);           
  } 
  stepper1.moveTo(X); 
  stepper1.run(); 

  // поворот второго сервопривода с помощью 4 енкодера
  // быстрое вращение и медленное вращение 
  // координаты автоматически фиксируются в памяти
  enc4.tick();
    if (enc4.isRight()) A--;        
    if (enc4.isLeft()) A++;         
    if (enc4.isFastR()) A -= 15;    
    if (enc4.isFastL()) A += 15;    
    if (enc4.isTurn())            
      Serial.println(A);            
    myservo2.write(A);

  // режим захвата объекта кнопкой 4 сервоприводом 1
  if (enc4.isClick()) {
    grabModeFlag *= -1;
      if (grabModeFlag == -1) {
        Serial.println(0);            
        myservo1.write(0);
        delay(500);
      }
      if (grabModeFlag == 1) {
        Serial.println(60);            
        myservo1.write(60);
        delay(500);
      }
  }  

  // переключение тумблера SB3 - вход в режим запоминания(кнопка 2 доступна для использования, кнопка 3 - не может использоваться)
  if(digitalRead(SB3)){
    memorizationModeFlag = 1;
    //lastState = 0;
    //Serial.println("Memorization mode is on.");
  }else {
    //Serial.println("Memorization mode is off.");
    memorizationModeFlag = 0;
  }

  // нажатие кнопки 2 - запоминание текущих координат манипулятора по всем осям; доступна только если режим запоминания = 1
  if(memorizationModeFlag)
    if(enc2.isClick()) {
      Serial.println("Storing state number " + (String) (lastState +  1) + '.');
      rememberState();
      Serial.println("Finished storing state number " + (String) lastState + '.');
    }
    
  // нажатие кнопки 3 - воспроизведение манипулятором маршрута из точек, заданных с помощью кнопки 2
  // доступна только если режим запоминания = -1 и если есть хотя бы одна точка для воспроизведения
  if(enc3.isClick() && lastState != 0 && memorizationModeFlag == 0)
    replayMemorized();
  enc5.tick();
   if(enc5.isClick()) {
      if(lastState != 0) {
        Serial.println("Вношу изменения во внутреннюю память");
        sendToEEPROM();
        Serial.println("Внутренняя память обновлена");
      }
   }

}
