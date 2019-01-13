#define  THRESHOLD 500
#define  S0     A4
#define  S1     A3
#define  S2     A2
#define  S3     A1
#define  S4     A0
#define  a1     8
#define  a2     9
#define  b1     12
#define  b2     13
#define  a_pwm  10
#define  b_pwm  11
int s[5],i;
String cmp_value;
//long OnTime = 250; 

void setup()
   {
    pinMode(a1,OUTPUT);
    pinMode(a2,OUTPUT);
    pinMode(b1,OUTPUT);
    pinMode(b2,OUTPUT);
    pinMode(a_pwm,OUTPUT);
    pinMode(b_pwm,OUTPUT);
    cmp_value="";
    
   }
     void loop()
     {
        unsigned long previousMillis = millis();
        Serial.println(previousMillis);
        
       while(1)
     {
      gofwd();
             if(cmp_value=="BBBBB")
             break;
          /*if(currentMillis - previousMillis >= OnTime)
          break;*/
     }
         unsigned long currentMillis = millis();
        Serial.println(currentMillis);
  
       /*goback();
       goright();
       goleft();*/
     }

     void readInput()
   {
     cmp_value="";
      s[0]=analogRead(S0);
      s[1]=analogRead(S1);
      s[2]=analogRead(S2);
      s[3]=analogRead(S3);
      s[4]=analogRead(S4);
       for(int i =0;i<5;i++)
        {
          if (s[i]>THRESHOLD)
                  {s[i]=1;cmp_value+="W";}
                  else {s[i]=0;cmp_value+="B";}
        }
        Serial.println(cmp_value);
   }
       void gofwd()
   {
   
     digitalWrite(a1,HIGH);
     digitalWrite(a2,LOW);
     digitalWrite(b1,HIGH);
     digitalWrite(b2,LOW);
     analogWrite(a_pwm,255);
     analogWrite(b_pwm,255);
     //delay(3000);
     readInput();
   
   }
   void goback()
   {
     digitalWrite(a1,LOW);
     digitalWrite(a2,HIGH);
     digitalWrite(b1,LOW);
     digitalWrite(b2,HIGH);
     analogWrite(a_pwm,255);
     analogWrite(b_pwm,255);
     delay(3000);
   }
   void goleft()

   {
     digitalWrite(a1,HIGH);
     digitalWrite(a2,LOW);
     digitalWrite(b1,HIGH);
     digitalWrite(b2,LOW);
     analogWrite(a_pwm,0);
     analogWrite(b_pwm,255);
     delay(3000);
   } 
  void goright()
   {
     digitalWrite(a1,HIGH);
     digitalWrite(a2,LOW);
     digitalWrite(b1,HIGH);
     digitalWrite(b2,LOW);
     analogWrite(a_pwm,255);
     analogWrite(b_pwm,0);
     delay(3000);
   } 
