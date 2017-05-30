#define RLAY_IGN_ARM 10
#define RLAY_IGN_PRI 9
#define RLAY_IGN_SEC 8

#define RF_VALVE_PWR 2
#define RF_VALVE_DIR 5
#define RV_VALVE_PWR 3
#define RV_VALVE_DIR 6
#define RDISCONN_PWR 4
#define RDISCONN_DIR 7

#define TOTAL_RLAYS 9
int relays[] = {
RLAY_IGN_ARM,
RLAY_IGN_PRI,
RLAY_IGN_SEC,
RF_VALVE_PWR,
RF_VALVE_DIR,
RV_VALVE_PWR,
RV_VALVE_DIR,
RDISCONN_PWR,
RDISCONN_DIR
};
byte global_tower_state = 0;
byte last_received_command = 0;
int state_received_index = 0;

//daq globals
#define DAQ_MASS_SENSOR A0
#define DAQ_PRESSURE_SENSOR A1
#define WINDOW_SIZE 20
int DAQ_MASS_HISTORY[WINDOW_SIZE];
int DAQ_PRESSURE_HISTORY[WINDOW_SIZE];
int DAQ_HISTORY_INDEX = 0;

//some constants
#define TIME_BETWEEN_ACK_REQUESTS 500
unsigned long time_last_ack_sent = 0;
#define TIME_LOST_CONTACT_IGNITION_SHUTOFF
#define TIME_LOST_CONTACT_TOTAL_SHUTOFF
unsigned long time_last_anything_received = 0;
enum{
    NO_IGNITION,
    SAFE_MODE,
    NORMAL_MODE
} safety_mode;

void setup(){
    Serial.begin(9600);
    for(int i = 0; i < TOTAL_RLAYS; i++){
        pinMode(relays[i], OUTPUT);
        digitalWrite(relays[i], LOW);
    }
}

void loop(){
    //get the readings from both DAQ sensors
    do{ //put in a dowhile for that sweet sexy vim syntax folding
    DAQ_MASS_HISTORY[DAQ_HISTORY_INDEX] =
        analogRead(DAQ_MASS_SENSOR);
    DAQ_PRESSURE_HISTORY[DAQ_HISTORY_INDEX] =
        analogRead(DAQ_PRESSURE_SENSOR);
    if(++DAQ_HISTORY_INDEX >= WINDOW_SIZE)
        DAQ_HISTORY_INDEX = 0;
    }while(0);

    //if there's an update from the client
    if(Serial.available()){
        byte input = Serial.read();
        switch(input){
            //if it's an ACK ('K')
            //if it's a State command ('U')
            //if it's a status request ('S')
            //if it's a hexit ('0'-'9' or 'A'-'F') 
        }
    }
}
