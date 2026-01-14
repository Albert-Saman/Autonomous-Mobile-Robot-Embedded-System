// --- GLOBAL VARIABLES ---
volatile unsigned int tick_ms = 0;      // Counts milliseconds
volatile unsigned char pwm_counter = 0; // divides the period into 100 sections
unsigned char mode = 1;                 // 1 = Stage 1 (Line), 2 = Stage 2 (Obstacle)

// --- STATE VARIABLES ---
unsigned char InTunnel = 0;
unsigned char HasEnteredTunnel = 0;
unsigned char intersectioncounter = 0;
unsigned char PostObstacle = 0;         // Flag to track if we passed the obstacle
volatile unsigned char peep = 0;        // Flag for beeping sound
volatile unsigned int beep_timer = 0;   // Timer for beep rhythm

// --- ISR SPEED CONTROL VARIABLES ---
volatile unsigned char speed_req_left = 0;
volatile unsigned char speed_req_right = 0;

unsigned int distance_cm;
unsigned int LDR_Value;
unsigned char line_left_val;
unsigned char line_right_val;

// --- CONFIGURATION CONSTANTS ---
#define TUNNEL_THRESHOLD 700 // Lower = Darker
#define TUNNEL_SPEED 45      // Speed in tunnel
#define MAX_SPEED 20         // Normal cruising speed
#define SLOW_SPEED 6         // Speed for pivot turns
#define TURN_SPEED 20        // Speed for line follower turns
#define OBSTACLE_DIST 20     // Detection distance
#define OBSTACLE_SPEED 35    // Speed in Obstacle Mode

// --- FUNCTION PROTOTYPES ---
void Setup_MCU(void);
void My_Delay_ms(unsigned int ms);
void MOTOR_SET_SPEED(unsigned int left_speed, unsigned int right_speed);
void MOTOR_STOP(void);
void MOTOR_PIVOT_LEFT(void);
void MOTOR_PIVOT_RIGHT(void);
void MOTOR_REVERSE(void);
void MOTOR_TURN_LEFT_LINE(void);
void MOTOR_TURN_RIGHT_LINE(void);
void Parking_Routine(void);
unsigned int ReadUltrasonicDistance();
unsigned int ReadLineLeft();
unsigned int ReadLineRight();
unsigned int ReadLDR();

// --- INTERRUPT SERVICE ROUTINE ---
void interrupt(void) {
    // Check TMR0 Interrupt Flag (Bit 2 of INTCON)
    if (INTCON & 0x04) {
        TMR0 = 6;
        tick_ms++;
        pwm_counter++;
        if (pwm_counter >= 100) pwm_counter = 0;

        // Left Motor (RC2 is Bit 2)
        if (pwm_counter < speed_req_left) PORTC |= 0x04;
        else PORTC &= ~0x04;

        // Right Motor (RC1 is Bit 1)
        if (pwm_counter < speed_req_right) PORTC |= 0x02;
        else PORTC &= ~0x02;

        // Beeping Logic (Rhythmic: 0.5s ON, 0.5s OFF)
        if (peep == 1) {
            beep_timer++;
            if (beep_timer >= 1000) beep_timer = 0; // Reset every 1 second (1000ms)

            if (beep_timer < 500) {
                // Sound ON (Toggle RC6 to generate tone)
                PORTC ^= 0x40;
            } else {
                // Sound OFF
                PORTC &= ~0x40;
            }
        } else {
            PORTC &= ~0x40;   // Ensure OFF when not peeping
            beep_timer = 0;
        }

        // Clear TMR0 Interrupt Flag (Bit 2)
        INTCON &= 0xFB;
    }
}

// --- MAIN FUNCTION ---
void main() {
    unsigned char current_speed;

    // Helpers for checking bits without variables
    unsigned char ir_left_active;
    unsigned char ir_right_active;

    Setup_MCU();

    // 1. Wait for Switch (RB0) to be turned ON
    // Checks if Bit 0 of PORTB is 1
    while(!(PORTB & 0x01));

    // 2. Delay 3 Seconds before starting
    My_Delay_ms(3000);

    // LED Indicator (RB1) ON to signal start
    PORTB |= 0x02;

    while(1){

        // Read Line Sensors
        line_left_val = ReadLineLeft();
        line_right_val = ReadLineRight();

        // ============================================
        // STAGE 1: LINE FOLLOWER + TUNNEL
        // ============================================
        if (mode == 1) {

            // --- TUNNEL DETECTION LOGIC ---
            LDR_Value = ReadLDR();

            if(LDR_Value < TUNNEL_THRESHOLD) {
                // Inside tunnel
                InTunnel = 1;
                HasEnteredTunnel = 1;
                PORTC |= 0x40;        // Buzzer ON (Bit 6) - Continuous in Tunnel
            }
            else {
                // Outside
                InTunnel = 0;
                if(peep == 0) PORTC &= ~0x40; // Buzzer OFF
            }

            // --- TRANSITION TO OBSTACLE MODE ---
            // This logic ensures it switches immediately upon exiting the tunnel
            if (HasEnteredTunnel == 1 && InTunnel == 0) {
                // Go LEFT immediately after leaving the tunnel
                MOTOR_PIVOT_LEFT();
                My_Delay_ms(500); // Turn duration (adjust if needed)

                MOTOR_STOP();
                PORTC &= ~0x40; // Ensure Buzzer OFF
                My_Delay_ms(500); // Stabilization pause

                mode = 2; // Switch to Obstacle Avoidance
                continue; // Skip rest of loop
            }

            // Set Speed
            current_speed = InTunnel ? TUNNEL_SPEED : MAX_SPEED;

            // Line Follower Logic
            if (line_right_val == 0 && line_left_val == 0) {
                // Forward
                if (PostObstacle) {
                    // Second Phase: Faster Speed
                    MOTOR_SET_SPEED(45, 45);
                } else {
                    // First Phase: Normal Speed
                    MOTOR_SET_SPEED(30, 30);
                }
            }
            else if (line_right_val == 0 && line_left_val == 1) {
                // Left Sensor on Line -> Adjust Right
                MOTOR_TURN_RIGHT_LINE();
            }
            else if (line_right_val == 1 && line_left_val == 0) {
                // Right Sensor on Line -> Adjust Left
                MOTOR_TURN_LEFT_LINE();
            }
            else if (line_right_val == 1 && line_left_val == 1) {
                // Intersection

                if(intersectioncounter == 0){
                    // First Intersection -> Sharp Pivot Left
                    speed_req_left = 95;
                    speed_req_right = 0;
                    PORTD = (PORTD & 0xF0) | 0x05;
                    My_Delay_ms(170);
                    intersectioncounter++;
                    MOTOR_STOP();
                    My_Delay_ms(500);
                }
                else if (intersectioncounter == 1|| intersectioncounter == 2 ||intersectioncounter == 3) {
                    // Ignore (Go Forward)
                    MOTOR_SET_SPEED(60, 60);
                    My_Delay_ms(50);
                    intersectioncounter++;
                }
                else {
                    // Parking
                    Parking_Routine();
                }
            }
        }

        // ============================================
        // STAGE 2: OBSTACLE AVOIDANCE
        // ============================================
        else if (mode == 2) {

            // --- TRANSITION BACK TO LINE FOLLOWER ---
            if (line_left_val == 1 || line_right_val == 1) {
                MOTOR_STOP();
                My_Delay_ms(500);

                // Reset Tunnel flags
                HasEnteredTunnel = 0;
                InTunnel = 0;
                intersectioncounter = 1;
                PostObstacle = 1; // Mark second phase active

                mode = 1; // Switch back to Line Follower
                continue;
            }

            // --- OBSTACLE AVOIDANCE LOGIC ---
            distance_cm = ReadUltrasonicDistance();

            // Check IR Sensors (Active Low, but typically 0 means detected)
            // RB4 is Right IR, RB5 is Left IR
            // Assuming 0 means "Obstacle Detected" for standard IR modules?
            // The original code had: IR_OBS_RIGHT == 1 for detected?
            // Let's stick to original logic:
            // Original: IR_OBS_RIGHT (RB4), IR_OBS_LEFT (RB5)

            ir_right_active = (PORTB & 0x10) ? 1 : 0; // Check Bit 4
            ir_left_active  = (PORTB & 0x20) ? 1 : 0; // Check Bit 5

            if (distance_cm > 0 && distance_cm < OBSTACLE_DIST) {
                MOTOR_STOP();
                My_Delay_ms(500);

                if (ir_left_active == 1 && ir_right_active == 0) {
                    MOTOR_TURN_LEFT_LINE();
                    My_Delay_ms(200);
                }
                else if (ir_right_active == 1 && ir_left_active == 0) {
                     MOTOR_TURN_RIGHT_LINE();
                    My_Delay_ms(200);
                }
                else if (ir_right_active == 1 && ir_left_active == 1) {
                    MOTOR_TURN_LEFT_LINE();
                    My_Delay_ms(300);
                }
                else {
                     MOTOR_TURN_LEFT_LINE();
                    My_Delay_ms(400);
                }
            }
            else {
                // IR Avoidance while moving
                if (ir_left_active == 0 && ir_right_active == 1) {
                       MOTOR_TURN_RIGHT_LINE();
                }
                else if (ir_right_active == 0 && ir_left_active == 1) {
                   MOTOR_TURN_LEFT_LINE();
                }
                else {
                    // Increased speed from MAX_SPEED(20) to OBSTACLE_SPEED(35)
                    MOTOR_SET_SPEED(OBSTACLE_SPEED, OBSTACLE_SPEED);
                }
                My_Delay_ms(50);
            }
        }
    }
}

// --- PARKING ROUTINE ---
void Parking_Routine(void) {
    unsigned char servo_i;

    MOTOR_STOP();
    My_Delay_ms(500);

    peep = 1; // Enable Beeping (Rhythmic)

    // Approach Wall
    distance_cm = ReadUltrasonicDistance();
    while(distance_cm > 20 || distance_cm == 0) {
        MOTOR_SET_SPEED(MAX_SPEED, MAX_SPEED);
        distance_cm = ReadUltrasonicDistance();
        My_Delay_ms(10);
    }
    MOTOR_STOP();
    My_Delay_ms(500);

    // Turn 90 Degrees Right
    MOTOR_PIVOT_RIGHT();
    My_Delay_ms(500);
    MOTOR_STOP();
    My_Delay_ms(500);

    // Pass through Bump / Enter Parking Area
    MOTOR_SET_SPEED(MAX_SPEED, MAX_SPEED);
    do {
        line_left_val = ReadLineLeft();
        line_right_val = ReadLineRight();
        My_Delay_ms(10);
    } while (!(line_left_val == 1 && line_right_val == 1));

    // Slow Down and Detect Wall
    distance_cm = ReadUltrasonicDistance();
    while(distance_cm > 10 || distance_cm == 0) {
        MOTOR_SET_SPEED(SLOW_SPEED, SLOW_SPEED);
        distance_cm = ReadUltrasonicDistance();
        My_Delay_ms(10);
    }

    // --- STOP AND FINISH ---
    MOTOR_STOP();

    // Disable beep logic, ensure buzzer OFF for silence, or keep it on?
    // User requested "buzzer in parking should beep every 1 second"
    // Usually this means WHILE parking. If you want it to beep AFTER parking forever,
    // keep peep = 1. If you want it to stop, set peep = 0.
    // Assuming "in the parking" means during the maneuver.
    // I will turn it off after parking is complete to save annoyance,
    // OR keep it on if "in the parking" means "while parked".
    // Standard behavior: Stop beeping when finished.
    peep = 0;
    PORTC &= ~0x40;      // Ensure Buzzer Off
    PORTB &= ~0x02;      // Turn OFF Status LED (RB1)

    // Raise Flag (Servo on RE2 - Bit 2)
    for(servo_i = 0; servo_i < 100; servo_i++) {
        PORTE |= 0x04;   // Set RE2 High
        My_Delay_ms(2);  // 2ms Pulse
        PORTE &= ~0x04;  // Set RE2 Low
        My_Delay_ms(18);
    }

    // Hold Flag Position (Keep generating 1ms pulses)
    while(1) {
        PORTE |= 0x04;
        My_Delay_ms(1);
        PORTE &= ~0x04;
        My_Delay_ms(19);
    }
}

// --- HELPER FUNCTIONS ---

void My_Delay_ms(unsigned int ms) {
    tick_ms = 0;
    while(tick_ms < ms);
}

void Setup_MCU(void) {
    // TRISA: RA0 Input (LDR) -> Bit 0 = 1
    TRISA = 0x01;

    // TRISB: RB0(Btn), RB2, RB3, RB4, RB5 Inputs -> 111101 -> 0x3D
    // RB1 Output (LED) -> 0
    TRISB = 0x3D;

    // TRISC: Output
    TRISC = 0x00;

    // TRISD: Output
    TRISD = 0x00;

    // TRISE: RE0(Trig)=Out(0), RE1(Echo)=In(1), RE2(Servo)=Out(0)
    // 00000010 -> 0x02
    TRISE = 0x02;

    PORTB = 0x00;
    PORTC = 0x00;
    PORTD = 0x00;
    PORTE = 0x00;

    // ADC Config: RA0 Analog, others Digital
    ADCON0 = 0x41; // ADC On, Channel 0, Fosc/8
    ADCON1 = 0xCE; // RA0 Analog, Result Right Justified

    // Timer0 Setup
    OPTION_REG = 0x82;
    TMR0 = 6;
    INTCON = 0xA0;

    // Timer1 Setup
    T1CON = 0x10;
}

unsigned int ReadUltrasonicDistance() {
    unsigned int timer_val;
    unsigned int timeout = 0;
    unsigned char d_i; // Loop variable

    // Trigger Pulse RE0 (Bit 0 of PORTE)
    PORTE |= 0x01;      // Set High

    // Manual Delay for ~10us
    // A simple loop. Adjust iteration count based on clock speed if needed.
    for(d_i = 0; d_i < 10; d_i++);

    PORTE &= ~0x01;     // Set Low

    TMR1H = 0; TMR1L = 0;

    // Wait for Echo High (RE1 is Bit 1 of PORTE)
    // While RE1 is 0...
    while((PORTE & 0x02) == 0) {
        timeout++;
        if(timeout > 30000) return 0;
    }

    // Turn ON Timer 1 (Bit 0 of T1CON)
    T1CON |= 0x01;

    // Wait for Echo Low (While RE1 is 1...)
    while((PORTE & 0x02) != 0) {
        if(TMR1H > 35) break;
    }

    // Turn OFF Timer 1
    T1CON &= ~0x01;

    // Calculate value without shifting (Rule compliance)
    // (TMR1H << 8) is equivalent to (TMR1H * 256)
    timer_val = (TMR1H * 256) + TMR1L;

    return (timer_val / 58);
}

unsigned int ReadLDR() {
    // Select Channel 0: Bits 5-3 of ADCON0 = 000
    // Keep ADC On (Bit 0) and Clock (Bits 7-6)
    ADCON0 &= 0xC5;
    ADCON0 |= 0x00;

    My_Delay_ms(2);      // Acquisition delay

    // Start Conversion: Set GO/DONE Bit (Bit 2 of ADCON0)
    ADCON0 |= 0x04;

    // Wait for completion: Check if GO/DONE Bit is High
    while(ADCON0 & 0x04);

    // Result
    return ((ADRESH * 256) + ADRESL);
}

// --- SENSOR READING ---
unsigned int ReadLineLeft() {
    // RB2 is Bit 2 (0x04)
    return (PORTB & 0x04) ? 1 : 0;
}

unsigned int ReadLineRight() {
    // RB3 is Bit 3 (0x08)
    return (PORTB & 0x08) ? 1 : 0;
}

// --- MOTOR CONTROLS ---

void MOTOR_SET_SPEED(unsigned int left_speed, unsigned int right_speed){
    PORTD = (PORTD & 0xF0) | 0x05;
    speed_req_left = left_speed;
    speed_req_right = right_speed;
}

void MOTOR_STOP(void) {
    speed_req_left = 0;
    speed_req_right = 0;
}

void MOTOR_PIVOT_LEFT(void) {
    PORTD = (PORTD & 0xF0) | 0x05;
    speed_req_left = 0;
    speed_req_right = MAX_SPEED;
}

void MOTOR_PIVOT_RIGHT(void) {
    PORTD = (PORTD & 0xF0) | 0x05;
    speed_req_left = MAX_SPEED;
    speed_req_right = 0;
}

void MOTOR_REVERSE(void) {
    PORTD = (PORTD & 0xF0) | 0x0A;
    speed_req_left = MAX_SPEED;
    speed_req_right = MAX_SPEED;
}

// UPDATED: Reverted to 0x06 (Left Back, Right Fwd)
void MOTOR_TURN_LEFT_LINE(void) {
    PORTD = (PORTD & 0xF0) | 0x06;
    speed_req_left = TURN_SPEED;
    speed_req_right = TURN_SPEED;
}

// UPDATED: Reverted to 0x09 (Left Fwd, Right Back)
void MOTOR_TURN_RIGHT_LINE(void) {
    PORTD = (PORTD & 0xF0) | 0x09;
    speed_req_left = TURN_SPEED;
    speed_req_right = TURN_SPEED;
}
