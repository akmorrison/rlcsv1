//#define DEBUG


#define BTN_IGN_ARM 3
#define BTN_IGN_PRI 4
#define BNT_IGN_SEC 5

#define SWI_RFILL_0 7
#define SWI_RFILL_1 8
#define SWI_RFILL_2 9
#define SWI_RFILL_3 10
#define SWI_RFILL_4 11
#define SWI_RFILL_5 12

#define TOTAL_BTNS 9
byte state_canon[] = {0,0,0,0,0,0,0,0,0};
byte state_temp[] = {0,0,0,0,0,0,0,0,0};
int buttons[] = { BTN_IGN_ARM, BTN_IGN_PRI, BNT_IGN_SEC, SWI_RFILL_0, SWI_RFILL_1, SWI_RFILL_2, SWI_RFILL_3, SWI_RFILL_4, SWI_RFILL_5};

#define command_length 6
int last_command_index = 0;
char last_command[] = {0,0,0,0,0,0};




void setup(){
    Serial.begin(9600);
    #ifdef DEBUG
    Serial.println("Hello World");
    #endif
    for(int i = 0; i < TOTAL_BTNS; i++)
        pinMode(buttons[i], INPUT);
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
    delay(100);
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
