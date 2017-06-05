//#define DEBUG_NETWORK
#include<LiquidCrystal.h>

#define BTN_IGN_PRI 12
#define BTN_IGN_SEC 11
#define RFILL_VALVE_PWR 7
#define RFILL_VALVE_DIR 4
#define RVENT_VALVE_PWR 6
#define RVENT_VALVE_DIR 3
#define RDISCONNECT_PWR 5
#define RDISCONNECT_DIR 2
#define TOTAL_BTNS 8
byte global_tower_state = 0; //holds 0 to 255
byte global_button_state[] = {0,0,0,0,0,0,0,0};
int buttons[] = {BTN_IGN_PRI, BTN_IGN_SEC, RFILL_VALVE_PWR, RFILL_VALVE_DIR, RVENT_VALVE_PWR, RVENT_VALVE_DIR, RDISCONNECT_PWR, RDISCONNECT_DIR};

long time_since_last_status; //when we last received a status
long time_since_last_status_request; //when we last asked for a status
#define TIME_BETWEEN_STATUS_REQUESTS 1000 //we received a status > 1000ms ago. We should ask for another one
#define TIME_BETWEEN_STATUS_WRITES 500 //we asked for a status 100ms ago, and didn't get anything. We should ask again
#define ACK_TIMEOUT 500
#define CONNECTION_LOST_TIME 5000

long time_last_command_sent = 0;
#define MIN_TIME_BETWEEN_COMMANDS 100

//LCD stuff
#define RS 8
#define RW 9
#define ENABLE 18
#define D4 14 //Analog pin 1's digital analog
#define D5 15
#define D6 16
#define D7 17
LiquidCrystal lcd(RS, RW, ENABLE, D4, D5, D6, D7);
//global holders for the daq data
byte global_tower_pressure[] = {'0','0','0','0'};
byte global_tower_mass[] = {'0','0','0'};

void setup(){
    Serial.begin(9600);
    for(int i = 0; i < TOTAL_BTNS; i++)
        pinMode(buttons[i], INPUT);

    time_since_last_status = millis();

    //lcd stuff
    lcd.begin(16,2);
    update_lcd();
}

enum { //states represent what the next expected byte will be
    NORMAL, //waiting for a ['K','S','P', or 'M'] command
    ACK, //waiting for state ack
    STATE, //waiting for state update
    PRESSURE, //waiting for pressure update
    MASS, //waiting for mass update
} global_expected_state;

byte state_in[] = {'0','0'}; //1st and second hexit of state
byte pressure_in[] = {'0','0','0','0'}; //4 bytes worth of pressure data
byte mass_in[] = {'0','0','0'}; //3 bytes worth of mass data
byte ack_in[] = {'0', '0'};
int data_in_index = 0;

void loop(){
    //read all the buttons, store in global_button_state
    for(int i = 0; i < TOTAL_BTNS; i++){
        global_button_state[i] = digitalRead(buttons[i]);
    }

    //if there's a message from the tower
    while(Serial.available()){
        byte input = Serial.read();
        #ifdef DEBUG_NETWORK
        put_char_on_lcd(input);
        #endif
        switch(input){
            case 'S': //tower is sending us a state update
                global_expected_state = STATE;
                time_since_last_status = millis();
                data_in_index = 0;
                break;
            case 'P': //tower is sending a pressure update
                global_expected_state = PRESSURE;
                data_in_index = 0;
                break;
            case 'M':
                global_expected_state = MASS;
                data_in_index = 0;
                break;
            case 'K': //tower is requesting an ack
                global_expected_state = ACK;
                data_in_index = 0;
                break;
            case '0':case '1':case '2':case '3':case '4':case '5':
            case '6':case '7':case '8':case '9':case 'A':case 'B':
            case 'C':case 'D':case 'E':case 'F': //tower can send hexits for mass, pressure, state, or ack requests
                if(global_expected_state == STATE){
                    if(data_in_index == 1){
                        //last hexit of state in
                        state_in[data_in_index] = input;
                        update_tower_state(); //just translate and store
                        data_in_index = 0; 
                        global_expected_state = NORMAL;
                    }
                    else
                        state_in[data_in_index++] = input; //store and increment
                }
                else if(global_expected_state == PRESSURE){
                    if(data_in_index == 3){
                        //last hexit of pressure in
                        pressure_in[data_in_index] = input;
                        update_pressure_data(); //translate and store
                        data_in_index = 0; //reset index
                        global_expected_state = NORMAL; //we're not waiting for anything now
                    }
                    else
                        pressure_in[data_in_index++] = input;
                }
                else if(global_expected_state == MASS){
                    if(data_in_index == 2){
                        //last hexit of mass in
                        mass_in[data_in_index] = input;
                        update_mass_data(); //translate and store
                        data_in_index = 0; //reset index
                        global_expected_state = NORMAL; //we're not waiting for anything now
                    }
                    else
                        mass_in[data_in_index++] = input;
                }
                else if(global_expected_state == ACK){
                    if(data_in_index == 1){
                        //last hexit of mass in
                        ack_in[data_in_index] = input;
                        //if ack in matches global_button_state, send
                        // back an ACK, otherwise send back a NACK
                        check_ack_and_respond();
                        data_in_index = 0; //reset index
                        global_expected_state = NORMAL; //we're not waiting for anything now
                    }
                    else
                        ack_in[data_in_index++] = input;
                }
                break;
        } //end switch(input)
    } //end Serial.available()


    //if global_button_state doesn't match global_tower_state
    if(global_tower_state != button_state_to_byte() && (millis()-time_last_command_sent) > MIN_TIME_BETWEEN_COMMANDS){
        //send a command to the tower
        send_button_state_to_tower();
    }

    //if it's been too long since we've gotten a status update
    if((millis()-time_since_last_status) > TIME_BETWEEN_STATUS_REQUESTS && (millis()-time_since_last_status_request) > TIME_BETWEEN_STATUS_WRITES){
        //request status update
        time_since_last_status_request = millis();
        write_to_xbee('S');
    }

    #ifndef DEBUG_NETWORK
    //update what's on the lcd
    update_lcd();
    #endif
}

void update_tower_state() {
//takes what's in state_in, stores in global_tower_state
//global_tower_state holds 1s or 0s. Not '1's or '0's
//step 1: convert the two bytes in state_in to single byte
    byte state_acc = 0;
    if(state_in[0] >= '0' && state_in[0] <= '9') //ascii assumption
        state_acc += (state_in[0] - '0'); //it's a decimal number
    else if(state_in[0] >= 'A' && state_in[0] <= 'F') //it's a hexit
        state_acc += (state_in[0] - ('A' - 10));
    else //didn't store hexits, something is fucked
        return;
    //multiply by 2^4, because state_in[0] is more significant
    state_acc <<= 4;

    //now deal with the second hexit
    if(state_in[1] >= '0' && state_in[1] <= '9') //ascii assumption
        state_acc += (state_in[1] - '0'); //it's a decimal number
    else if(state_in[1] >= 'A' && state_in[1] <= 'F') //it's a hexit
        state_acc += (state_in[1] - ('A' - 10));
    else //didn't store hexits, something is fucked
        return;

    //now that we have a proper byte, store in global
    global_tower_state = state_acc;
}

//takes what's in pressure_in, stores in global_tower_pressure
void update_pressure_data(){
//this is just a simple copy operation
    for(int i = 0; i < 4; i++)
        global_tower_pressure[i] = pressure_in[i];
}

//takes what's in mass_in, stores in global_tower_mass
void update_mass_data(){
//just a simple copy operation
    for(int i = 0; i < 3; i++)
        global_tower_mass[i] = mass_in[i];
}

//compares what's in ack_in to what's in global_button_state
void check_ack_and_respond(){
//if they match, send an ACK, (which is a 'K'), otherwise,
//send back a NACK (which is an 'N')
    //first, convert the 2 hexits in ack_in to a single byte.
    //copy pasted from update_tower_state
    byte state_acc = 0;
    if(ack_in[0] >= '0' && ack_in[0] <= '9') //ascii assumption
        state_acc += (ack_in[0] - '0'); //it's a decimal number
    else if(ack_in[0] >= 'A' && ack_in[0] <= 'F') //it's a hexit
        state_acc += (ack_in[0] - ('A' - 10));
    else //didn't store hexits, something is fucked
        return;
    //multiply by 2^4, because ack_in[0] is more significant
    state_acc <<= 4;

    //now deal with the second hexit
    if(ack_in[1] >= '0' && ack_in[1] <= '9') //ascii assumption
        state_acc += (ack_in[1] - '0'); //it's a decimal number
    else if(ack_in[1] >= 'A' && ack_in[1] <= 'F') //it's a hexit
        state_acc += (ack_in[1] - ('A' - 10));
    else //didn't store hexits, something is fucked
        return;

    //we now have an acc byte. If it matches global_button_state, send 'K'
    //otherwise, send 'N'
    if(button_state_to_byte() == state_acc){
        write_to_xbee('K');
        //if we're acking, we should then immediately check if it's been applied
        write_to_xbee('S');
    }
    else
        write_to_xbee('N');
}

//translates global_button_state to tower
void send_button_state_to_tower(){
    byte to_send = button_state_to_byte();
    byte first_byte = (to_send & 0b11110000) >> 4;
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

    //the 'U' flag means "this is an update"
    time_last_command_sent = millis();
    write_to_xbee('U');
    write_to_xbee(first_byte);
    write_to_xbee(second_byte);
}

//converts what's in global_button_state to a single byte
byte button_state_to_byte(){
    byte accumulator = 0;
    if(global_button_state[0]) accumulator += (1 << 7);
    if(global_button_state[1]) accumulator += (1 << 6);
    if(global_button_state[2]) accumulator += (1 << 5);
    if(global_button_state[3]) accumulator += (1 << 4);
    if(global_button_state[4]) accumulator += (1 << 3);
    if(global_button_state[5]) accumulator += (1 << 2);
    if(global_button_state[6]) accumulator += (1 << 1);
    if(global_button_state[7]) accumulator += (1 << 0);
    return accumulator;
}

enum{
    LCD_NORMAL,
    LCD_UPDATING,
    LCD_CONN_LOST
} lcd_state;
unsigned long seconds_conn_lost = 0;
//changes what's on the LCD
unsigned long lcd_normal_counter = 0;
void update_lcd(){
    //if it's been longer than CONECTION LOST TIME since last contact
    if((millis()-time_since_last_status)>CONNECTION_LOST_TIME){
        //find out if we need to update
        if(lcd_state == LCD_CONN_LOST && seconds_conn_lost == ((millis()-time_since_last_status)/1000))
            //we apparently don't need to update
            return;
        lcd.clear();
        lcd.print("SIGNAL LOST");
        lcd.setCursor(0,1);
        lcd.print("LAST CONTACT ");
        lcd.print((seconds_conn_lost = ((millis()-time_since_last_status)/1000)));
        lcd.write('S');
        lcd_state = LCD_CONN_LOST;
    }
    //if the global_tower_state doesn't match button state
    else if(button_state_to_byte() != global_tower_state){
        //do we not need to update
        if(lcd_state == LCD_UPDATING)
            return; //no update needed
        lcd.clear();
        lcd.print("SENDING UPDATE");
        lcd.setCursor(0,1);
        lcd.print("TO TOWER");
        lcd_state = LCD_UPDATING;
    }
    //if everything is great
    else{
        if(lcd_normal_counter++ < 200 && lcd_state == LCD_NORMAL)
            return;
        lcd_normal_counter = 0;
        lcd.clear();
        //output pressure data
        lcd.print("PRESSURE:");
        lcd.write(global_tower_pressure[0]);
        lcd.write(global_tower_pressure[1]);
        lcd.write(global_tower_pressure[2]);
        lcd.write(global_tower_pressure[3]);
        lcd.print("PSI");
        //output mass data
        lcd.setCursor(0,1);
        lcd.print("MASS: ");
        lcd.write(global_tower_mass[0]);
        lcd.write(global_tower_mass[1]);
        lcd.write('.');
        lcd.write(global_tower_mass[2]);
        lcd.print("LBS");
        lcd_state = LCD_NORMAL;
    }
}

#ifdef DEBUG_NETWORK
int lcd_x = 0;
void put_char_on_lcd(char in){
    lcd_x++;
    if(lcd_x >= 16){
        lcd_x = 0;
    }

    lcd.setCursor(lcd_x,0);
    lcd.write(in);
}

int lcd_out_x = 0;;
void put_out_char_on_lcd(char in){
    lcd_out_x++;
    if(lcd_out_x >= 16){
        lcd_out_x = 0;
    }

    lcd.setCursor(lcd_out_x,1);
    lcd.write(in);
}
#endif

void write_to_xbee(char out){
    #ifdef DEBUG_NETWORK
    put_out_char_on_lcd(out);
    #endif
    Serial.write(out);
}
