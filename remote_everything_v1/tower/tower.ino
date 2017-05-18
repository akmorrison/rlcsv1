#define DEBUG


#define RLAY_IGN_ARM 12
#define RLAY_IGN_PRI 11
#define RLAY_IGN_SEC 10

#define RLAY_RFILL_0 3
#define RLAY_RFILL_1 6
#define RLAY_RFILL_2 4
#define RLAY_RFILL_3 7
#define RLAY_RFILL_4 5
#define RLAY_RFILL_5 8

#define TOTAL_RLAYS 9
int relays[] = {
 RLAY_IGN_ARM,
 RLAY_IGN_PRI,
 RLAY_IGN_SEC,
 RLAY_RFILL_0,
 RLAY_RFILL_1,
 RLAY_RFILL_2,
 RLAY_RFILL_3,
 RLAY_RFILL_4,
 RLAY_RFILL_5
};
byte state_canon[] = {0,0,0,0,0,0,0,0,0};
byte state_received[] = {0,0,0,0,0,0,0,0,0};
int state_received_index = 0;

void setup(){
    Serial.begin(9600);
    #ifdef DEBUG
    Serial.println("Hello World");
    #endif
    for(int i = 0; i < TOTAL_RLAYS; i++){
        pinMode(relays[i], OUTPUT);
        digitalWrite(relays[i], LOW);
    }
}

byte c;
void loop(){
    if(Serial.available() > 0){
        c = Serial.read();
        #ifdef DEBUG
        Serial.write(c);
        #endif
        if(c == 'S') //send a status report
            //Serial.println("S received");
            send_status_report();
        else if(c == '0' || c == '1')
            state_received[state_received_index++ % TOTAL_RLAYS] = c - '0';
        else if(c == '\r'){ //client wants to write out to relays
            send_ack_and_write();
            state_received_index = 0;
            for(int i = 0; i < TOTAL_RLAYS; i++)
                state_received[i] = 0;
            while(Serial.available()) Serial.read();
        }
    }
    delay(100);
}

void send_status_report(){
    #ifdef DEBUG
    Serial.print("Status: ");
    #else
    Serial.write('S');
    #endif
    for(int i = 0; i < TOTAL_RLAYS; i++)
        Serial.write(state_canon[i] + '0');
    #ifdef DEBUG
    Serial.write('\n');
    Serial.write('\r');
    #endif
}

void send_ack_and_write(){
    #ifdef DEBUG
    Serial.write('\n');
    Serial.write('\r');
    #endif
    Serial.write("ACK", 3);
    for(int i = 0; i < TOTAL_RLAYS; i++)
        Serial.write(state_received[i] + '0');
    while(!Serial.available())
        delay(10);
    if(Serial.read() == 'A'){
        for(int i = 0; i < TOTAL_RLAYS; i++){
            state_canon[i] = state_received[i];
            if(state_canon[i])
                digitalWrite(relays[i], HIGH);
            else
                digitalWrite(relays[i], LOW);
        }
        #ifdef DEBUG
        Serial.write('\n');
        Serial.write('\r');
        #endif
    }
}
