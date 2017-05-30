//#define DEBUG


#define RLAY_IGN_ARM 10
#define RLAY_IGN_PRI 9
#define RLAY_IGN_SEC 8

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

#define DAQ_MASS_SENSOR A0
#define DAQ_PRESSURE_SENSOR A1
#define WINDOW_SIZE 20
int DAQ_MASS_HISTORY[WINDOW_SIZE];
int DAQ_PRESSURE_HISTORY[WINDOW_SIZE];
int DAQ_HISTORY_INDEX = 0;

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
    while(Serial.available() > 0){
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
    //read DAQ values
    DAQ_MASS_HISTORY[DAQ_HISTORY_INDEX] = analogRead(DAQ_MASS_SENSOR);
    DAQ_PRESSURE_HISTORY[DAQ_HISTORY_INDEX] = analogRead(DAQ_PRESSURE_SENSOR);
    if(++DAQ_HISTORY_INDEX >= WINDOW_SIZE)
        DAQ_HISTORY_INDEX = 0;
}

void send_status_report(){
    #ifdef DEBUG
    Serial.print("Status: ");
    #else
    Serial.write('S');
    #endif
    for(int i = 0; i < TOTAL_RLAYS; i++)
        Serial.write(state_canon[i] + '0');

    //report the daq readings
    int pressure = analogRead(DAQ_PRESSURE_SENSOR);
    int mass = analogRead(DAQ_MASS_SENSOR);
   
    #ifdef DEBUG
    Serial.print("Pressure: ");
    #else
    Serial.write('P');
    #endif
    serial_write_pressure_reading();

    #ifdef DEBUG
    Serial.print("Mass: ");
    #else
    Serial.write('M');
    #endif
    serial_write_mass_reading();

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

#define DAQ_READING_LENGTH 4
int serial_write_pressure_reading(){
    long accumulator = 0;
    for(int i = 0; i < WINDOW_SIZE; i++)
        accumulator += DAQ_PRESSURE_HISTORY[i];
    accumulator /= WINDOW_SIZE;

    //the math for pressure sensor is said to be
    //P1 = Vin*610.55, +46.965
    accumulator *= 611 * 5;
    accumulator >>= 10;
    accumulator += 47;

    Serial.print((accumulator / 1000)% 10); //print the 4th digit
    Serial.print((accumulator / 100) % 10);
    Serial.print((accumulator / 10 ) % 10);
    Serial.print(accumulator % 10); //print the first digit
}

int serial_write_mass_reading(){
    long accumulator = 0;
    for(int i = 0; i < WINDOW_SIZE; i++)
        accumulator += DAQ_MASS_HISTORY[i];
    accumulator /= WINDOW_SIZE;

    //The math for load cell is said to be
    //LC = Vin*25.0, -12.734
    accumulator *= 25 * 5;
    accumulator >>= 10;
    accumulator -= 12;

    if(accumulator < 0 || accumulator > 100)
        Serial.print("ERR");
    else{
        Serial.print((accumulator / 1000)% 10); //print the 4th digit
        Serial.print((accumulator / 100) % 10);
        Serial.print((accumulator / 10 ) % 10);
        Serial.print(accumulator % 10); //print the first digit
    }
}
