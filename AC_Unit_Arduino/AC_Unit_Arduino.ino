#include <LiquidCrystal.h>
#include <boarddefs.h>
#include <IRremoteInt.h>
#include <ir_Lego_PF_BitStreamEncoder.h>
#include <IRremote.h>

const int RECV_PIN = 6;
IRrecv irrecv(RECV_PIN);
decode_results results;

LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

int ac_temp;
int ac_status=0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  irrecv.enableIRIn();
  irrecv.blink13(true);

  lcd.begin(16, 2);
  lcd.print("AC Status : OFF");

}

void loop() {
  // put your main code here, to run repeatedly:
  if (irrecv.decode(&results)){
        int x=results.value;
        if(x==18615)
        {
          ac_temp=20;
          ac_status=1;
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("AC Status : ON");
          lcd.setCursor(0, 1);
          lcd.print(String("AC Temp : "+String(ac_temp)));
          
        }
        if(x==30855)
        {
          lcd.clear();
          ac_status=0;
          lcd.setCursor(0, 0);
          lcd.print("AC Status : OFF");
        }
        if(x==-24481 && ac_status==1)
        {
          ac_temp--;
          ac_temp=max(ac_temp,16);
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("AC Status : ON");
          lcd.setCursor(0, 1);
          lcd.print(String("AC Temp : "+String(ac_temp)));
        }
        if(x==24735 && ac_status==1)
        {
          ac_temp++;
          ac_temp=min(ac_temp,30);
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("AC Status : ON");
          lcd.setCursor(0, 1);
          lcd.print(String("AC Temp : "+String(ac_temp)));
        }
        irrecv.resume();
  }
  
}
