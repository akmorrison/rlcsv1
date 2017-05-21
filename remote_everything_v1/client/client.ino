//#define DEBUG

#include<LiquidCrystal.h>

#define BTN_IGN_ARM 12
#define BTN_IGN_PRI 11
#define BNT_IGN_SEC 10

#define SWI_RFILL_0 5
#define SWI_RFILL_1 6
#define SWI_RFILL_2 7
#define SWI_RFILL_3 4
#define SWI_RFILL_4 3
#define SWI_RFILL_5 2

#define TOTAL_BTNS 9
byte state_canon[] = {0,0,0,0,0,0,0,0,0}; //holds either 0 or 1. Not '0' or '1'
byte state_temp[] = {0,0,0,0,0,0,0,0,0};
int buttons[] = { BTN_IGN_ARM, BTN_IGN_PRI, BNT_IGN_SEC, SWI_RFILL_0, SWI_RFILL_1, SWI_RFILL_2, SWI_RFILL_3, SWI_RFILL_4, SWI_RFILL_5};

unsigned long time_since_last_status;
#define MILLIS_BETWEEN_STATUS 1000

//LCD stuff
#define RS 8
#define RW 9
#define ENABLE 18
#define D4 14 //Analog pin 1's digital analog
#define D5 15
#define D6 16
#define D7 17
#define PRESSURE_LEADER "PRESSURE:"
#define PRESSURE_START_POINT 9,0
#define MASS_LEADER "GRAVITY:"
#define MASS_START_POINT 8,1
LiquidCrystal lcd(RS, RW, ENABLE, D4, D5, D6, D7);

void setup(){
    Serial.begin(9600);
    #ifdef DEBUG
    Serial.println("Hello World");
    #endif
    for(int i = 0; i < TOTAL_BTNS; i++)
        pinMode(buttons[i], INPUT);

    time_since_last_status = millis();

    //lcd stuff
    lcd.begin(16,2);
    setup_lcd();
}


void loop(){
    byte send_update = 0;
    for(int i = 0; i < TOTAL_BTNS; i++){
        state_temp[i] = digitalRead(buttons[i]);
        if(state_temp[i] != state_canon[i])
            send_update = 1;
    }
    if(send_update){
        for(int i = 0; i < TOTAL_BTNS; i++)
            Serial.write(state_temp[i] + '0');
        #ifdef DEBUG
        Serial.write('\n');
        #endif
        flush_serial_input();
        Serial.write('\r');
        if(rec_ack()){
            #ifdef DEBUG
            Serial.println("ack received");
            #endif
            for(int i = 0; i < TOTAL_BTNS; i++)
                state_canon[i] = state_temp[i];
        }
        #ifdef DEBUG
        else
            Serial.println("ack not received");
        #endif
    }

    if(millis() - time_since_last_status > MILLIS_BETWEEN_STATUS){
        get_status();
    }
    delay(100);
}

#define DAQ_READING_LENGTH 4
bool error_bit;
void get_status(){
    delay(100);
    flush_serial_input();
    Serial.write('S');
    if(getbyte() != 'S')
        error_bit = true;
    for(int i = 0; i < TOTAL_BTNS; i++){
        byte input = getbyte();
        if(input < '0' || input >= '9')
            error_bit = true;
        state_canon[i] = input - '0';
    }
    
    setup_lcd();
    //print out the pressure data
    if(getbyte() != 'P')
        error_bit = true;
    lcd.setCursor(PRESSURE_START_POINT);
    for(int i = 0; i < DAQ_READING_LENGTH; i++)
        lcd.write(getbyte());
    
    //print out the mass data
    if(getbyte() != 'M')
        error_bit = true;
    lcd.setCursor(MASS_START_POINT);
    for(int i = 0; i < DAQ_READING_LENGTH; i++)
        lcd.write(getbyte());
}

void flush_serial_input(){
    while(Serial.available())
        Serial.read();
}

byte getbyte(){
    while(!Serial.available())
        delay(10);
    return Serial.read();
}

int rec_ack(){
    while(!Serial.available())
        delay(10);
    if(getbyte() != 'A' || getbyte() != 'C' || getbyte() != 'K')
        return 0;
    for(int i = 0; i < TOTAL_BTNS; i++){
        byte c = getbyte();
        #ifdef DEBUG
        Serial.write(c);
        #endif
        if((c-'0') != state_temp[i]){
            Serial.write('N');
            return 0;
        }
    }
    Serial.write('A'); //Ack that this is correct
    return 1;
}

void setup_lcd(){
    lcd.clear();
    lcd.print(PRESSURE_LEADER);
//    lcd.setCursor(PRESSURE_START_POINT);
//    lcd.print("UNKNOWN");
    lcd.setCursor(13,0);
    lcd.print("PSI");
    lcd.setCursor(0,1);
    lcd.print(MASS_LEADER);
//    lcd.setCursor(MASS_START_POINT);
//    lcd.print("UNKNOWN");
    lcd.setCursor(13,1);
    lcd.print("LBS");
}
