// Драйвер малого мотора 1600 и 3200 шагов = 1 об = 16mm, Nema Драйвер 1600 и 1600 шагов = 1 об.
#include <arduino.h>
#include <LiquidCrystal_I2C.h>
//#include <GyverButton.h>
#include <EncButton.h>
//#include "GyverPlanner2.h"          // радиус круга
#include "timer.h"
#include "GyverPlanner.h"
#define CLK 8 //enc
#define DT 9  // enc
int_fast32_t pos = 0; // enc
byte lastState = 0; // enc
const int8_t increment[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0}; // enc
Stepper<STEPPER2WIRE> stepper1(2, 5);   //Стол
Stepper<STEPPER2WIRE> stepper2(3, 6);   //Шпиндель
GPlanner<STEPPER2WIRE, 2> planner;
int sensStop = 8;   //датчик кз
int sensorPin = A0; // пока не использую
int homePin = 11;   //Два датчика Дом Стол
int ztable = 0;     // flag z table
void zt();  //declarate function
int32_t sensorValue = 0;
int32_t Pos1 = 0;
int32_t Pos2 = 0;
LiquidCrystal_I2C lcd(0x27, 16, 2); // Устанавливаем дисплей
EncButton<EB_TICK, 8, 9, 10> enc;    // энкодер с кнопкой <A, B, KEY>
#define ENC_TYPE 0                   // тип энкодера, 0 или 1


void setup() {
  pinMode(homePin, INPUT);           // назначить выводу порт ввода
  digitalWrite(homePin, HIGH);       // включить подтягивающий резистор

  //Serial.begin(9600);
  lcd.init();
  lcd.backlight(); // Включаем подсветку дисплея

  // добавляем шаговики на оси
  planner.addStepper(0, stepper1);  // ось 0
  planner.addStepper(1, stepper2);  // ось 1

  // устанавливаем ускорение и скорость
  planner.setAcceleration(3000);
  planner.setMaxSpeed(30000);
  zt ();   // table to zero
}

byte count = 0;
int32_t path[][2] = {
  {0, 0},
  {0, 0},
  {0, 0},
};

// Funtions 
  void zt () 
      {
        while (digitalRead(homePin)==1)
            {
            planner.setSpeed( 0 ,3000);     // Goto Zero Speed 3000
            planner.tick();                 // tick manual
            }
        planner.brake();                    // STOP
        planner.reset();                    // RESET ALL AXIS
        ztable = 1; // table zero pozition initialisated
        planner.setAcceleration(500);      //Set Acc
        planner.setMaxSpeed(3000);         //Set Speed
        /*while (true != (planner.ready())) // переделать на 2 датчика!!!!!!!!!!!!!!!!!!!!!
            {
              Pos1 = -200;   // Стол откатить от Zero
              Pos2 = -200;    // Шпиндель
              int32_t path[][2] = {
                  {Pos1, Pos2},

              };
              planner.setTarget(path[count]); // загружаем новую точку (начнётся с 0)
              if (++count >= sizeof(path) / 8)  count = 0;
            
            planner.resume();        
            planner.tick();
            lcd.setCursor(11, 0);
            planner.tick();
            lcd.print(planner.getCurrent(1));
            }*/
            pos = 100;        //откатиться на  исходную
      }

     //----------------------------------------------------------------- 
  void loop()
  {
  byte state = digitalRead(CLK) | (digitalRead(DT) << 1); //Encoder
  if (state != lastState)
      {
        pos += increment[state | (lastState << 2)];           //Encoder
        lastState = state;
      }
   
   
    // здесь происходит движение моторов, вызывать как можно чаще
                // Control Zero
    /*if(digitalRead(homePin)==0)  {
      planner.brake();
      planner.reset();
      pos = 0;
    }  
    while (true != (planner.ready())) // переделать на 2 датчика!!!!!!!!!!!!!!!!!!!!!
    {
      Pos1 = 200;   // Стол откатить от Zero
      Pos2 = 0;    // Шпиндель
      int32_t path[][2] = {
          {Pos1, Pos2},

      };
      planner.setTarget(path[count]); // загружаем новую точку (начнётся с 0)
      if (++count >= sizeof(path) / 8)  count = 0;*/
    
    planner.resume();        
    planner.tick();
    //}
    // вернёт true, если все моторы доехали
    if (planner.ready())
    {
      /*if (digitalRead(sensStop) == 0) {
      sensorValue = sensorValue - 1;
    }
      else  {
      sensorValue = sensorValue + 1;
    }*/
      sensorValue = pos*50;
      // sensorValue = датчик КЗ sensStop pin 2 ... временно , перевести в режим компоратора или ацп
      Pos1 = sensorValue * 2.0;   // Стол
      Pos2 = sensorValue * 1.6;    // Шпиндель
      int32_t path[][2] = {
          {Pos1, Pos2},

      };
      planner.setTarget(path[count]); // загружаем новую точку (начнётся с 0)
      if (++count >= sizeof(path) / 8)  count = 0;
    }

    // управляем процессом
    /* if (Serial.available() > 0) {
       char incoming = Serial.read();
       switch (incoming) {
         case 's': planner.stop(); break;
         case 'b': planner.brake(); break;
         case 'r': planner.resume(); break;
         case 'p': planner.pause(); break;
       }
     }*/

    // асинхронно вывожу в порт графики
    static uint32_t tmr;
    if (millis() - tmr >= 500)
    {
      tmr = millis();
      lcd.setCursor(0, 0);
      planner.tick();
      lcd.print(1);
      lcd.setCursor(0, 1);
      planner.tick();
      lcd.print(2);
      lcd.setCursor(3, 0);
      planner.tick();
      lcd.print(stepper1.pos);
      lcd.setCursor(3, 1);
      planner.tick();
      lcd.print(stepper2.pos);
      lcd.setCursor(11, 0);
      planner.tick();
      lcd.print(planner.getCurrent(1));
      lcd.setCursor(11, 1);
      planner.tick();
      lcd.print(pos/2);
    }
  }
  