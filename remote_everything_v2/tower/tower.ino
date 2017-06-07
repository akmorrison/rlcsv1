//used only for debugging when plugged into a laptop
//comment this line out before uploading for realsies
//#define DEBUG

//safest possible state
//both ignitions off, both valves directions off and power on
#define SAFEST_POSSIBLE_STATE 0b00101000


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

//things for the status LEDS
#define STATUS_LED_OK   11
#define STATUS_LED_WARN 12
#define STATUS_LED_RED  13

//daq globals
#define DAQ_MASS_SENSOR A0
#define DAQ_PRESSURE_SENSOR A1
#define WINDOW_SIZE 20
int DAQ_MASS_HISTORY[WINDOW_SIZE];
int DAQ_PRESSURE_HISTORY[WINDOW_SIZE];
int DAQ_HISTORY_INDEX = 0;
byte global_tower_mass[] = {'0','0','0'};
byte global_tower_pressure[] = {'0','0','0','0'};

//some constants
#define TIME_BETWEEN_ACK_REQUESTS 500
unsigned long global_time_last_ACK_request = 0;
#define TIME_LOST_CONTACT_IGNITION_SHUTOFF 3000
#define TIME_LOST_CONTACT_TOTAL_SHUTOFF 10000
unsigned long global_time_last_contact;
enum{
    NO_IGNITION,
    SAFE_MODE,
    NORMAL_MODE
} safety_mode = NORMAL_MODE;

void setup(){
    Serial.begin(9600);
    for(int i = 0; i < TOTAL_RLAYS; i++){
        pinMode(relays[i], OUTPUT);
        digitalWrite(relays[i], LOW);
    }
    //setup the status leds
    pinMode(STATUS_LED_OK, OUTPUT);
    pinMode(STATUS_LED_WARN, OUTPUT);
    pinMode(STATUS_LED_RED, OUTPUT);
    digitalWrite(STATUS_LED_OK, HIGH);
    digitalWrite(STATUS_LED_WARN, LOW);
    digitalWrite(STATUS_LED_RED, LOW);
    global_time_last_contact = millis();
}

enum{ //what we're waiting on
    INPUT_NORMAL, //not waiting on anything
    INPUT_COMMAND //waiting on an input hexit
} input_state;
byte command_raw[] = {'0', '0'}; //last two command bytes received
int command_index = 0;

//when this is true, we are waiting for an ack. Only ever set true
//by send_ack. marked false by receive_ack and when we receive a new command.
//when set false, do not change the relays
bool global_waiting_for_ack = false;

void loop(){
    //get the readings from both DAQ sensors
    DAQ_MASS_HISTORY[DAQ_HISTORY_INDEX] =
        analogRead(DAQ_MASS_SENSOR);
    DAQ_PRESSURE_HISTORY[DAQ_HISTORY_INDEX] =
        analogRead(DAQ_PRESSURE_SENSOR);
    if((++DAQ_HISTORY_INDEX) >= WINDOW_SIZE)
        DAQ_HISTORY_INDEX = 0;

    //if there's an update from the client
    if(Serial.available()){
        byte input = Serial.read();
        switch(input){
            //if it's an ACK ('K')
            case 'K':
                receive_ack();
                break;
            //if it's a State command ('U')
            case 'U':
                input_state = INPUT_COMMAND;
                command_index = 0;
                global_waiting_for_ack = false;
                break;
            //if it's a status request ('S')
            case 'S':
                send_full_status_update();
                break;
            //if it's a hexit ('0'-'9' or 'A'-'F') 
            case'0':case'1':case'2':case'3':case'4':case'5':
            case'6':case'7':case'8':case'9':case'A':case'B':
            case'C':case'D':case'E':case'F':
                if(input_state == INPUT_COMMAND){
                    if(command_index == 1){
                        command_raw[command_index] = input;
                        send_ack();
                        input_state = INPUT_NORMAL;
                        command_index = 0;
                    }
                    else if(command_index == 0)
                        command_raw[command_index++] = input;
                }
        }
    }

    //if we're waiting for an ACK from the client
    if(global_waiting_for_ack && (millis()-global_time_last_ACK_request) > TIME_BETWEEN_ACK_REQUESTS){
        //it's taken too long for the client to respond. Ask again
        send_ack();
    }

    //if we've lost contact with the client for too long: turn off ignition
    if(safety_mode == NORMAL_MODE && (millis()-global_time_last_contact > TIME_LOST_CONTACT_IGNITION_SHUTOFF)){
        //turn off the ignition
        goto_no_ignition_mode();
    }

    //if we've lost contact with the client for way too long: back to normal
    if(safety_mode == NO_IGNITION && ((millis()-global_time_last_contact) > TIME_LOST_CONTACT_TOTAL_SHUTOFF)){
        //turn all valves to safe mode
        goto_full_safety();
    }
}

//we have received an ack. copy command_raw to global_tower_state
void receive_ack(){
    //assume that we have received an ack
    //which means we're A), no longer waiting on one
    global_waiting_for_ack = false;
    //and B), we've received contact
    global_time_last_contact = millis();
    if(safety_mode != NORMAL_MODE){
        //we've received contact, we can enter normal mode
        goto_normal_mode();
    }
    //copy command_raw over to global_tower_state
    byte first_byte, second_byte;
    if(command_raw[0] >= '0' && command_raw[0] <= '9'){ //first byte is dec
        first_byte = command_raw[0] - '0';
    }
    else if(command_raw[0] >= 'A' && command_raw[0] <= 'F'){
        //first byte is a hexit
        first_byte = command_raw[0] - ('A' - 10);
    }
    //now do the second byte
    if(command_raw[1] >= '0' && command_raw[1] <= '9'){ //first byte is dec
        second_byte = command_raw[1] - '0';
    }
    else if(command_raw[1] >= 'A' && command_raw[1] <= 'F'){
        //first byte is a hexit
        second_byte = command_raw[1] - ('A' - 10);
    }
    
    //assign first byte and second byte to global_tower_state.
    //we are assuming that both bytes are < 16.
    global_tower_state = (first_byte << 4) + second_byte;
    apply_state_to_relays();
}

//we have received a full command. make sure the client wants it
void send_ack(){
//we know that the last command is in command_raw
//reset global_time_last_ACK_request
    global_time_last_ACK_request = millis();
    write_to_xbee('K');
    write_to_xbee(command_raw[0]);
    write_to_xbee(command_raw[1]);
}

//send back a status update featuring tower state and daq readings
//this should only be called when the client requests a status
//this function resets global_time_last_contact. DO NOT CALL ON A TIMER.
void send_full_status_update(){
    global_time_last_contact = millis(); //we were asked for status.
                                         //being asked is a contact
    if(safety_mode != NORMAL_MODE){
        //we've received contact, we can leave our safety mode
        goto_normal_mode();
    }
    //send the status
    byte to_send = global_tower_state;
    byte first_byte = (to_send & 0b11110000) >> 4; //take the first half
    byte second_byte= to_send & 0b00001111;
    //now we have a value, need to convert to ascii
    if(first_byte < 10) //it's a digit
        first_byte += '0';
    else{ //it's a hexit
        first_byte -= 10;
        first_byte += 'A';
    }
    //do the same thing with the second byte
    if(second_byte < 10) //it's a digit
        second_byte += '0';
    else{ //it's a hexit
        second_byte -= 10;
        second_byte += 'A';
    }

    //the 'S' flag means "this is a status update"
    write_to_xbee('S');
    write_to_xbee(first_byte);
    write_to_xbee(second_byte);

    //send the pressure data
    calc_daq_values_and_store();
    write_to_xbee('P');
    for(int i = 0; i < 4; i++)
        write_to_xbee(global_tower_pressure[i]);
    //send the mass data
    write_to_xbee('M');
    for(int i = 0; i < 3; i++)
        write_to_xbee(global_tower_mass[i]);
}

//We've heard from the operator. Good to to to normal safety mode
void goto_normal_mode(){
    #ifdef DEBUG
    Serial.println("GOING TO NORMAL");
    #endif
    safety_mode = NORMAL_MODE;
    digitalWrite(STATUS_LED_OK, HIGH);
    digitalWrite(STATUS_LED_WARN, LOW);
    digitalWrite(STATUS_LED_RED, LOW);
}
//we haven't heard from the operator in a while, turn off the ignition
//relays if they're on
void goto_no_ignition_mode(){
    #ifdef DEBUG
    Serial.println("GOING TO NO IGNITITION");
    #endif
    //if any ignition relays are on
    if(global_tower_state & 0b11000000){
        global_tower_state &= 0b00111111;//turn off the ignition relays
        apply_state_to_relays(); //apply changes
    }
    safety_mode = NO_IGNITION;
    digitalWrite(STATUS_LED_OK, LOW);
    digitalWrite(STATUS_LED_WARN, HIGH);
    digitalWrite(STATUS_LED_RED, LOW);
}

//the operator can no longer tell us what to do
//go to the safest mode possible
void goto_full_safety(){
    #ifdef DEBUG
    Serial.println("GOING TO FULL SAFE");
    #endif
    //TODO, talk to yash/miranda. Find out what the safest possible mode is. 
    global_tower_state = SAFEST_POSSIBLE_STATE;
    apply_state_to_relays();
    safety_mode = SAFE_MODE;
    digitalWrite(STATUS_LED_OK, LOW);
    digitalWrite(STATUS_LED_WARN, LOW);
    digitalWrite(STATUS_LED_RED, HIGH);
}

//whatever's in the global_tower_state, put it on the relays
//Note, this should only be called immediately after receiving an ack
void apply_state_to_relays(){
    /*
    if(safety_mode == NO_IGNITION || safety_mode == SAFE_MODE){
        //we're in a safety mode. We don't change a thing
        //technically, this shouldn't be possible, as receive_ack()
        //resets your safety state. Anyways though, defensive programming
        return;
    }
    */
    //ignition relays
    bool ign_pri =      ((global_tower_state & 0b10000000) != 0);
    bool ign_sec =      ((global_tower_state & 0b01000000) != 0);
    //remote fill relays
    bool rfill_pwr =    ((global_tower_state & 0b00100000) != 0);
    bool rfill_dir =    ((global_tower_state & 0b00010000) != 0);
    bool rvent_pwr =    ((global_tower_state & 0b00001000) != 0);
    bool rvent_dir =    ((global_tower_state & 0b00000100) != 0);
    bool rdisconn_pwr = ((global_tower_state & 0b00000010) != 0);
    bool rdisconn_dir = ((global_tower_state & 0b00000001) != 0);

    //we are now changing the relays.
    //if we want to burn, we need to turn on the arm switch
    digitalWrite(RLAY_IGN_ARM, (ign_pri || ign_sec));
    digitalWrite(RLAY_IGN_PRI, ign_pri);
    digitalWrite(RLAY_IGN_SEC, ign_sec);
    //now change the remote fill relays
    digitalWrite(RF_VALVE_PWR, rfill_pwr);
    digitalWrite(RF_VALVE_DIR, rfill_dir);
    digitalWrite(RV_VALVE_PWR, rvent_pwr);
    digitalWrite(RV_VALVE_DIR, rvent_dir);
    digitalWrite(RDISCONN_PWR, rdisconn_pwr);
    digitalWrite(RDISCONN_DIR, rdisconn_dir);
    //done
}

//take the data stored in DAQ_PRESSURE_HISTORY and DAQ_MASS_HISTORY
//translate to bytes. Store in global_tower_pressure and global_tower_mass
void calc_daq_values_and_store(){
    long mass_accumulator = 0, pressure_accumulator = 0;
    for(int i = 0; i < WINDOW_SIZE; i++){
        mass_accumulator += DAQ_MASS_HISTORY[i];
        pressure_accumulator += DAQ_PRESSURE_HISTORY[i];
    }
    mass_accumulator /= WINDOW_SIZE;
    pressure_accumulator /= WINDOW_SIZE;

    //normalize the pressure data
    //the math for pressure sensor is said to be
    //P1 = Vin*610.55 + 46.965
    pressure_accumulator *= 611 * 5;
    pressure_accumulator >>= 10;
    pressure_accumulator += 47;
    global_tower_pressure[0] = ((pressure_accumulator /1000) %10) + '0';
    global_tower_pressure[1] = ((pressure_accumulator /100) %10) + '0';
    global_tower_pressure[2] = ((pressure_accumulator /10) %10) + '0';
    global_tower_pressure[3] = ((pressure_accumulator) %10) + '0';

    //normalize the mass data
    //The math for the load cell is said to be
    //LC = Vin*25 - 12.734
    //Since resoldering the connector, we need to subtract an additional 11.3 lbs
    //we're multiplying that by 10 because we want 0.1LBS resolution
    mass_accumulator *= 25 * 5;
    mass_accumulator /= 102; //not 1024 (arduino analog scale) because res
    mass_accumulator -= 125;
    if(mass_accumulator < 0){
        //there's a negative mass on the mass sensor
        //we set to 0 because client only accepts hexits here
        global_tower_mass[0] = 'L';
        global_tower_mass[1] = 'T';
        global_tower_mass[2] = '0';
    }
    else if(mass_accumulator >= 1000){
        //more than 100LBS on the mass sensor (what?)
        //we set to 99.9 because client wouldn't accept letters
        global_tower_mass[0] = '9';
        global_tower_mass[1] = '9';
        global_tower_mass[2] = '9';
    }
    else{
        //we're good
        global_tower_mass[0] = ((mass_accumulator /100) %10) + '0';
        global_tower_mass[1] = ((mass_accumulator /10) %10) + '0';
        global_tower_mass[2] = ((mass_accumulator) %10) + '0';
    }
}

void write_to_xbee(char out){
    Serial.write(out);
//    delay(50);
}
