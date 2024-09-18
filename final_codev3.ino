
//YWROBOT
//Compatible with the Arduino IDE 1.0
//Library version:1.1

#include <AccelStepper.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#define dirPin 2
#define stepPin 3
#define motorInterfaceType 1
#define PWM_pin 6
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);
int pot,pot_p,val1=0,val2=0;
uint16_t Dm;
double T0=25+273.15,R0=100000,Rb=10000,beta=3950;
double Tk=0,Tc=0,R=0;
int erreur=0,P_erreur=0;
int T_pwm=0,E=0,i=0;
float elapsedTime, Time, timePrev;
int kp = 27;   int ki = 0.6 ;   int kd = 0.6;
int PID_p = 0;    int PID_i = 0;    int PID_d = 0;
int Mset=0;
int Tset=40;
String data = "";
LiquidCrystal_I2C lcd(0x27,16,2);  


void setup()
{
  lcd.init();  // initialize the lcd 
  lcd.backlight();
  pinMode(10, INPUT);
  pinMode(11, INPUT);
  pinMode(PWM_pin,OUTPUT); //pwm output
  stepper.setMaxSpeed(1000);
  Time = millis();
  lcd.setCursor(0,0);
  lcd.print("  Config.T&S ");

}

void loop()
{
  Tc=thermistor();
  pot=analogRead(A0);
  val1 = digitalRead(10);   // read the input pin
  val2 = digitalRead(11);   // read the input pin
  
  if (val1==1){
  i=1;
  stepper.stop();
  pot=map(pot,0,1023,10,1000);
  lcd.setCursor(0,0);
  lcd.print("Motor Speed:      ");
  if (pot!=pot_p) {
    lcd.setCursor(0,1);
    lcd.print("(R.12)            ");
    lcd.setCursor(9,1);
    lcd.print(map(pot,10,1000,1,100));
    lcd.setCursor(12,1);
    lcd.print("%");
    Mset=pot;
    pot_p=pot;
  }
  }
  else if (val2==1){
  i=1;
  stepper.stop();
  pot=map(pot,0,1023,20,300);
  lcd.setCursor(0,0);
  lcd.print("Val temperature:      ");
  if (pot!=pot_p) {
    
    lcd.setCursor(0,1);
    lcd.print("(R.260)                    ");
    lcd.setCursor(9,1);
    lcd.print(pot);
    lcd.setCursor(12,1);
    lcd.print(" C");
    pot_p=pot;
    Tset=pot;
  }
  }
  else {
    if (i==1){
    i=0;
    lcd.setCursor(0,0);
    lcd.print("Temperature :      ");
    lcd.print("                   ");
    lcd.setCursor(0,1);
    lcd.print("                   ");
    lcd.setCursor(8,1);
    lcd.print(int(Tc));
    }
    stepper.setSpeed(Mset);
    updateDisplay();
    stepper.runSpeed();
  analogWrite(PWM_pin,T_pwm);
  P_erreur=E;
  }
erreur=(Tset*1.034)-Tc;
P_erreur=0;
  if(erreur<= 0){erreur=0;}
  E=map(erreur,0,int(Tc),0,255);
  PID_p=E*kp;
  if(-3 < E <3) 
  {
  PID_i = PID_i + (ki * E);
  }
  timePrev = Time;                            
  Time = millis();                           
  elapsedTime = (Time - timePrev) / 1000; 
  //Now we can calculate the D calue
  PID_d = kd*((E-P_erreur)/elapsedTime);
  T_pwm= PID_p + PID_i + PID_d;
   if(T_pwm < 0)
  {    T_pwm  = 0;    }
  else if(T_pwm  > 255)  
  {    T_pwm  = 255;}  

  analogWrite(PWM_pin,T_pwm);
  P_erreur=E; 
}
double thermistor()
{
  int t = analogRead(A2);
  float tr = 1023.0 / t - 1;
  tr = Rb / tr;
   float steinhart;
    steinhart = tr / R0;
    steinhart = log(steinhart);
    steinhart /= beta;
    steinhart += 1.0 / (T0);
    steinhart = 1.0 / steinhart;
    steinhart -= 273.15; 
  return steinhart;
}
void updateDisplay()
{
   static unsigned long timer = 0;
   unsigned long interval = 3000;  // update display 2 times / second
   if (millis() - timer >= interval)
   {
      timer = millis();
      lcd.print("      "); // overwrite old data
      lcd.setCursor(8,1);  // reset cursor
      lcd.print(Tc);

   }
}
