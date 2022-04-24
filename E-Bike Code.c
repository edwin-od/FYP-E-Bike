
//PIC18F4550 Configuration

#pragma config PLLDIV = 5				// prescaler is not used but is still set for completeness
#pragma config CPUDIV = OSC1_PLL2		// postscaler is not used but is still set for completeness
#pragma config USBDIV = 2				// USB clock selection is not used but is still set for completeness

#pragma config FOSC = INTOSCIO_EC		// internal clock with pin14 as IO pin and pin13

#pragma config FCMEN = OFF				// fail-safe monitor disabled
#pragma config IESO = OFF				// internal/external oscillator switchover bit disabled
#pragma config PWRT = OFF				// power-up timer disabled
#pragma config BOR = OFF				// brown-out reset disabled in hardware and software
#pragma config BORV = 3					// brown-out reset voltage bits - does not matter since voltage disabled
#pragma config VREGEN = OFF				// USB voltage regulator disabled
#pragma config WDT = OFF				// watchdog timer disabled
#pragma config WDTPS = 32768			// watchdog timer postscale - does not matter since watchdog timer is disabled
#pragma config CCP2MX = ON				// RC1 as CCP2 mux
#pragma config PBADEN = OFF				// RBs set as digital pins
#pragma config LPT1OSC = OFF			// timer 1 as oscillator disabled
#pragma config MCLRE = OFF				// master clear disabled//////////////////////////////////////////////
#pragma config STVREN = ON				// stack full - underflow causes reset
#pragma config LVP = OFF				// ICSP disabled
#pragma config ICPRT = OFF				// ICPORT disabled
#pragma config XINST = OFF				// instruction set extension disabled
#pragma config CP0 = OFF, CP1 = OFF, CP2 = OFF, CP3 = OFF				// code protection bits OFF
#pragma config CPB = OFF				// boot block code protection OFF
#pragma config CPD = OFF				// data EEPROM code protection OFF
#pragma config WRT0 = OFF, WRT1 = OFF, WRT2 = OFF, WRT3 = OFF			// write protection bits OFF
#pragma config WRTC = OFF				// config registers write protection OFF
#pragma config WRTB = OFF				// boot block write protection OFF
#pragma config WRTD = OFF				// data EEPROM write protection OFF
#pragma config EBTR0 = OFF, EBTR1 = OFF, EBTR2 = OFF, EBTR3 = OFF		// table read protection bits OFF
#pragma config EBTRB = OFF				// boot block table read protection OFF

//Includes

#include<p18f4550.h>
#include<delays.h>

// Global Constants
#pragma udata constants
const unsigned char pwm_period_register = 249;
const unsigned char pwm_break_thresh = 85; // in %
const double potentiometer_to_pwm_CONST = 0.097752; // 100/1023
const unsigned char speedometer_calibration_recheck_steps = 3;	// this will be the number of steps taken away and towards the stopper to make sure we are at 0 on the speedometer scale at the end of the calibration process
//const unsigned int gear_stepper_step_size = 200;	//200steps=180deg
const double samples_timeout_in_s = 2.5; // 2.5 or 5.0
const float K = 0.311;		// PID constant
const unsigned char target_speed_reading_error_margin = 2;	// target_speed will be considered the same in the range [target-margin, target+margin]
const unsigned char gear_error_margin = 20;	// correct gear will be set between [target-err, target+err]
const unsigned int gear_1 = 0;				// gear 1 analog reading from gear position slider(10k potentiometer)
const unsigned int gear_2 = 256;			// gear 2 analog reading from gear position slider(10k potentiometer)
const unsigned int gear_3 = 512;			// gear 3 analog reading from gear position slider(10k potentiometer)
const unsigned int gear_4 = 768;			// gear 4 analog reading from gear position slider(10k potentiometer)
const unsigned int gear_5 = 1023;			// gear 5 analog reading from gear position slider(10k potentiometer)
const unsigned int gear_1_thresh_KMPH = 3;				// gear 1 threshold speed
const unsigned int gear_2_thresh_KMPH = 6;			// gear 2 threshold speed
const unsigned int gear_3_thresh_KMPH = 8;			// gear 3 threshold speed
const unsigned int gear_4_thresh_KMPH = 12;			// gear 4 threshold speed
const unsigned int gear_5_thresh_KMPH = 15;			// gear 5 threshold speed
const unsigned int cadence_threshold = 10;	// RPM threshold when we assume that below it the user is pedaling lightly, otherwise the user is pedaling aggressively
const double RPH_CONST = 900000000; 	// (1,000,000s x 3,600h) / 4us  --> (i.e. 4x10^(-6) to convert to seconds, and x3,600 to convert to hours) /->timer3
const unsigned short long RPM_CONST = 468750;// each timer increment is (4x256)/8,000,000 = 128us --to convert to minutes--> 128/(1,000,000 x 60) and to convert to frequency(i.e. RPM) (1,000,000x60)/128 = 468,750 /->timer0
const unsigned short long RPS_CONST = 250000; // timer3 -> (1,000,000s/4us) = 250,000
const float circumference = 0.002042;	// considering radius is 32.5cm, circumference = 2 x pi x 32.5cm x 10^(-5) = 0.002042km  --> (i.e. 10^(-5) to convert to km)
const unsigned char max_wheel_overflow_count = 7;		// how many wheel revolution count overflows are allowed before considering that the bike stopped (0 kmph) (max value for this constant = 64)
							// max elapsed t = (65,535 + [65,535 x max(overflows)]) x (1/[8,000,000/(4 x prescaler)])
							// (4 with 8 prescaler -> a little over 1 second elapsed) (20 with 8 prescaler ->  5.505 seconds elapsed)

	
// Global Variables
#pragma udata globals
unsigned int gear_position;						// AN0 reading
unsigned int dash_gear_pot;						// AN1 reading
unsigned int dash_motor_pot;					// AN2 reading
signed char current_speedometer_step;			// current step position if speedometer


// volatile global variables (i.e. used interrupts subroutine)
#pragma udata volatiles
volatile unsigned char wheel_overflow_count;				// how many wheel revolution count overflows happened
volatile unsigned long wheel_rev_elapsed_timer_count;		// (65535 x wheel_overflow_count) + wheel_rev_timer_val <- if wheel_overflow_count != 0
volatile unsigned char cruise_emergency_stop;				// boolean indicating that the breaks have been used, and cruise control should be disengaged
volatile double cur_KMPH;									// current speed in KMPH
volatile double prv_KMPH;									// speed in KMPH of the first sample after cycle reset of the pid processing
volatile unsigned char cadence;								// current cadence
volatile double previous_dynamic_target_speed;				// stored speed of last cruise control speed set
volatile double total_pid_integration;						// accumulated integration of the pid controller
volatile double total_sampling_time_elapsed;				// accumulated time from the last samples processing cycle




// Interrupts Subroutine

#pragma interrupt isr //save=PROD
void isr(void);

// Function Declarations

void normalize_duty_cycle(unsigned char duty_cycle_percentage);
void process_speed_sample(void);
void process_and_update_PID(void);
void read_AN0(void);
void read_AN1(void);
void read_AN2(void);
unsigned int calculate_automated_gear(void);
void update_gear_stepper(unsigned int target_gear);
void update_speedometer_stepper(void);
void calibrate_speedometer_stepper(signed char limit);
void gear_stepper_step(unsigned char direction);
void speedometer_stepper_step(unsigned char direction);


// Main Function + Infinite Loop

void main(void) {
	
	// Initializations
	
	OSCCONbits.IDLEN = 0b0;				// sleep mode when SLEEP is called
	OSCCONbits.IRCF = 0b111;			// 8MHz INTOSC selected as is
	OSCCONbits.SCS = 0b10;				// internal oscillator enabled

	while(!OSCCONbits.IOFS) {}			// wait until internal clock is stable
	
	TRISBbits.TRISB0 = 0b1;				// set RB0/INT0 pin as input as external interrupt
	TRISBbits.TRISB1 = 0b1;				// set RB1/INT1 pin as input as external interrupt
	
	INTCONbits.GIE = 0b1;				// enable external global interrupts

	INTCONbits.INT0IE = 0b1;			// enable external interrupt pin INT0
	INTCON2bits.INTEDG0 = 0b0;			// set external interrupt INT0 as falling edge triggered for wheel hall effect sensor detection

	INTCON3bits.INT1IE = 0b1;			// enable external interrupt pin INT1
	INTCON2bits.INTEDG1 = 0b0;			// set external interrupt INT1 as falling edge triggered for cadence near fall effect sensor detection
	
	INTCONbits.INT0IF = 0b0;			// clear interrupt0 flag
	INTCON3bits.INT1IF = 0b0;			// clear interrupt1 flag
	
	INTCONbits.PEIE_GIEL = 0b1;			// enable peripheral interrupts for timers
	
	T3CONbits.RD16 = 0b1;				// set timer3 as 16-bit timer
	T3CONbits.TMR3CS = 0b0;				// set timer3 clock source from internal clock (Fosc/4)
	T3CONbits.T3CKPS = 0b11;			// set timer3 presecaler value to 1:8
	T3CONbits.TMR3ON = 0b0;				// set timer3 initially OFF
	PIE2bits.TMR3IE = 0b1;				// set timer3 overflow interrupt enabled
	PIR2bits.TMR3IF = 0b0;				// clear timer3 overflow interrupt flag initally
	TMR3H = 0x00;						// clear timer3 higher nibble
	TMR3L = 0x00;						// clear timer3 lower nibble
	
	wheel_overflow_count = 0;			// set wheel revolution timer overflow count to 0 initally
	wheel_rev_elapsed_timer_count = 0;	// set revolution total elapsed time to 0 initially
	cur_KMPH = 0;						// set current speed to 0 initially
	prv_KMPH = 0;						// set previous speed to 0 initially
	cadence = 0;						// set current cadence 0 initially
	total_pid_integration = 0;			// set integration 0 initially
	total_sampling_time_elapsed = 0;	// set total sampling time 0 initially
	previous_dynamic_target_speed = 0;	// set previous target speed 0 initially
	cruise_emergency_stop = 0;			// cruise emegency stop boolean initially False
	

	T0CONbits.T08BIT = 0b0;				// set timer0 to 16 bit timer
	T0CONbits.T0CS = 0b0;				// set timer0 clock source as internal clock(Fosc/4)
	T0CONbits.T0SE = 0b0;				// set timer0 as rising edge increment timer
	T0CONbits.PSA = 0b0;				// enable timer0 prescaler
	T0CONbits.T0PS = 0b111;				// set timer0 prescaler as 256
	T0CONbits.TMR0ON = 0b0;				// timer0 OFF initially
	INTCONbits.TMR0IE = 0b1;			// enable timer0 overflow interrupt
	INTCONbits.TMR0IF = 0b0;			// clear timer0 overflow interrupt flag initally
	TMR0H = 0x00;						// clear timer0 higher nibble
	TMR0L = 0x00;						// clear timer0 lower nibble
	
	
	TRISCbits.TRISC2 = 0b0;				// set Main Motor PWM pin as output
	PR2 = pwm_period_register;			// set PWM frequency as 10KHz (-> [8,000,000/(10,000 x 4 x 1)]-1)
	normalize_duty_cycle(0);			// set duty cycle initially setting the motor OFF
	T2CON = 0b00000010;					// setup Timer 2 -> No postscaler, initally OFF, prescaler set to 1
	TMR2 = 0;							// clear Timer 2 initally
	CCP1CON = 0b00001100;				// setup CCP1 module -> Single Output mode, .0 duty cycle decimal, PWM mode (P1A & P1B are active-high)
	T2CONbits.TMR2ON = 0b1;				// turn Timer 2 ON, thus starting the PWM output
	
	// A/D resolution is 10 bits (i.e. [0, 1023])
	ADCON0bits.ADON = 0b0;				// analog to digital converter OFF initially
	ADCON0bits.GO_NOT_DONE = 0b0;		// analog to digital converter not in progress initially
	TRISAbits.RA0 = 0b1;				// set RA0/AN0 as input pin
	TRISAbits.RA1 = 0b1;				// set RA1/AN1 as input pin
	TRISAbits.RA2 = 0b1;				// set RA2/AN2 as input pin
	ADCON1bits.PCFG = 0b1100;			// set AN0, AN1 and AN2 as analog pins
	ADCON1bits.VCFG = 0b00;				// set voltage references as VDD and VSS
	ADCON2bits.ADFM = 0b1;				// set A/D output as right justified 
	ADCON2bits.ADCS = 0b101;			// set A/D clock to Fosc/16 (i.e. Tad = 2us > 0.8us which is the minimum period required by the datasheet)
	ADCON2bits.ACQT = 0b001;			// set acquisition time to 2xTad (i.e. 4us > 2.45us which is the minimum acquisition time required by the datasheet)
	ADRESH = 0;							// clear A/D output higher nibble register initially
	ADRESL = 0;							// clear A/D output lower nibble register initially
	
	gear_position = 0;					// AN0 reading initially 0
	dash_gear_pot = 0;					// AN1 reading initially 0
	dash_motor_pot = 0;					// AN2 reading initially 0
	
	TRISDbits.TRISD6 = 0b0;				// set gear shift stepper motor direction as output
	LATDbits.LATD6 = 0b0;				// set gear shift stepper motor direction OFF initially
	
	TRISDbits.TRISD4 = 0b0;				// set gear shift stepper motor step as output
	LATDbits.LATD4 = 0b0;				// set gear shift stepper motor step OFF initially
	
	
	TRISAbits.TRISA3 = 0b1;				// set assist mode selection switch pin as input
	TRISAbits.TRISA4 = 0b1;				// set cruise mode selection switch pin as input
	
	TRISDbits.TRISD3 = 0b1;				// set rear brake flag as input
										// set front brake flag as input - RC4 does not need to be set as input pins since it are always input
	
	TRISDbits.TRISD2 = 0b1;				// set speedometer stop flag as input (used for calibration)			
	
	TRISAbits.TRISA5 = 0b0;				// speedometer stepper (white) -> closest to power supply input on the driver board
	TRISEbits.TRISE0 = 0b0;				// speedometer stepper (brown)
	TRISEbits.TRISE1 = 0b0;				// speedometer stepper (white)
	TRISEbits.TRISE2 = 0b0;				// speedometer stepper (grey)
	LATAbits.LATA5 = 0b0;				// initially OFF
	LATEbits.LATE0 = 0b0;				// initially OFF
	LATEbits.LATE1 = 0b0;				// initially OFF
	LATEbits.LATE2 = 0b0;				// initially OFF
	
	calibrate_speedometer_stepper(40);			// calibrate speedometer position and set current speedometer step 0 initially with limit 40 steps (38 is the whole visible range of the speedometer)
	
	
	while(1) {			// Infinite Loop
		
		unsigned char temp = 0;
		
		if(PORTAbits.RA3) {								// Assist Control
			
			//Find-out if currently pedaling
			if(cadence != 0) {
				
				read_AN2();		// manual motor speed
				normalize_duty_cycle(dash_motor_pot * potentiometer_to_pwm_CONST);
				
				// Automated gear shifting relative to bike speed (KMPH) and pedaling cadence (RPM) (assuming front derailleur is set to the middle disk)
				update_gear_stepper(calculate_automated_gear());
				
			} else {
				normalize_duty_cycle(0);
			}
			
			if(cruise_emergency_stop > 0) {
				cruise_emergency_stop = 0;		// reset cruise emergency boolean
			}
		} 
		
		else if(PORTAbits.RA4) {					// Cruise Control
			
			// In this mod of operation the motor PWM is adjusted automatically through interrupts subroutine and the PID control processing method
			
			read_AN1();		// manual gear shifting
			update_gear_stepper(dash_gear_pot);
			
		}
		
		else {										// Manual Control
			
			read_AN2();		// manual motor speed
			normalize_duty_cycle(dash_motor_pot * potentiometer_to_pwm_CONST);
			
			read_AN1();		// manual gear shifting
			update_gear_stepper(dash_gear_pot);
			
			if(cruise_emergency_stop > 0) {
				cruise_emergency_stop = 0;		// reset cruise emergency boolean
			}
			
		}
		
		
		update_speedometer_stepper();
		
	}
	
}


#pragma code interrupt_vector = 0x08
void isr() {

	// Wheel Hall Effect Sensor Detection
	if(INTCONbits.INT0IF) {					// check if INT0 interrupt flag is high
			
		unsigned char low;
		
		unsigned int wheel_rev_timer_val;
		
		low = TMR3L;							// load lower nibble into a temp variable to initiate high nibble transfer to the buffer
		wheel_rev_timer_val = TMR3H;			// load higher nibble into the final timer variable
		
		TMR3H = 0x00;						// clear higher nibble
		TMR3L = 0x00;						// clear lower nibble
		
		if(!T3CONbits.TMR3ON) {
			T3CONbits.TMR3ON = 0b1;				// set timer3 ON to restart cycle before carrying with the calculations needed to process the speed
		}
		
		wheel_rev_timer_val = wheel_rev_timer_val << 8;		// shift the lower 8 bits (and replace with 0) in place of the higher 8 bits
		wheel_rev_timer_val += low;							// add lower nibble to complete the whole 16 bits of the timer count
		
		wheel_rev_elapsed_timer_count = (65535 * wheel_overflow_count) + wheel_rev_timer_val;		// calculate total elapsed timer counts from last revolution
		
		process_speed_sample();
		
		wheel_overflow_count = 0;
		
		INTCONbits.INT0IF = 0b0;			// reset INT0 interrupt flag for next cycle
		
	} else if(PIR2bits.TMR3IF) {
		
		if(wheel_overflow_count > max_wheel_overflow_count) {			// check if overflow counter has exceeded maximum overflows	(i.e. we can consider that the bike stopped)
			
			prv_KMPH = 0;
			cur_KMPH = 0;
			total_pid_integration = 0;
			previous_dynamic_target_speed = 0;
			
			wheel_overflow_count = 0;
			wheel_rev_elapsed_timer_count = 0;
			
			T3CONbits.TMR3ON = 0b0;				// set timer3 OFF
			TMR3H = 0x00;						// clear higher nibble
			TMR3L = 0x00;						// clear lower nibble
			
		} else {
		
			wheel_overflow_count = wheel_overflow_count + 1;		// increment overflow counter to allow low frequency revolutions
			
		}
		
		PIR2bits.TMR3IF = 0b0;				// reset timer3 overflow interrupt flag
	} 
	
	// Pedal Hall Effect Sensor Detection	
	else if(INTCON3bits.INT1IF) {			// check if INT1 interrupt flag is high
		
		unsigned char low;
		unsigned int cadence_period;
		
		low = TMR0L;							// load lower nibble into a temp variable to initiate high nibble transfer to the buffer
		cadence_period = TMR0H;			// load higher nibble into the final timer variable
		
		TMR0H = 0x00;						// clear timer0 higher nibble
		TMR0L = 0x00;						// clear timer0 lower nibble
		
		if(!T0CONbits.TMR0ON) {
			T0CONbits.TMR0ON = 0b1;				// timer0 ON if pedalled and timer was off
		}
		
		cadence_period = cadence_period << 8;		// shift the lower 8 bits (and replace with 0) in place of the higher 8 bits
		cadence_period += low;							// add lower nibble to complete the whole 16 bits of the timer count
		
		if(cadence_period != 0) {
			cadence = RPM_CONST / cadence_period;
		}				
		
		INTCON3bits.INT1IF = 0b0;			// reset INT1 interrupt flag for next cycle
		
	} else if(INTCONbits.TMR0IF) {				// check if timer0 overflow interrupt flag is high
		
		cadence = 0;
		
		T0CONbits.TMR0ON = 0b0;				// timer0 OFF
		
		INTCONbits.TMR0IF = 0b0;
		
	}
}

void normalize_duty_cycle(unsigned char duty_cycle_percentage) {

	if(duty_cycle_percentage == 255) {
		CCPR1L = pwm_period_register;	// BREAK
		return;
	}

	duty_cycle_percentage = 100 - duty_cycle_percentage; // H-Bridge used needs the pwm flipped: 100% -> 0V & (0+)% -> 36V
	if(duty_cycle_percentage > pwm_break_thresh) {
		duty_cycle_percentage = pwm_break_thresh;
	}
	
	CCPR1L = ((double)duty_cycle_percentage * ((double)pwm_period_register / 100.0));
	
}


void process_speed_sample(void) {
	
	if(wheel_rev_elapsed_timer_count != 0){	
		
		double rev_time_in_s = wheel_rev_elapsed_timer_count / (double)RPS_CONST; 	
		unsigned int RPH = RPH_CONST / wheel_rev_elapsed_timer_count;
		
		total_sampling_time_elapsed += rev_time_in_s;
		
		if(total_sampling_time_elapsed <= samples_timeout_in_s) {
			
			cur_KMPH = RPH * circumference;
			
			if(prv_KMPH == 0) {
				prv_KMPH = cur_KMPH;
			}
			
		} else {
			
			process_and_update_PID();
			prv_KMPH = RPH * circumference;
		}
		
		wheel_rev_elapsed_timer_count = 0;
	}
}

void process_and_update_PID(void) {
	
	if(PORTAbits.RA4) {							// check that cruise control is chosen
		
		double pid;
		
		double total_pid_proportional;			// saved locally in memory for convinient debugging. Can be removed to free up memory usage and reduce loading and storing operations
		double total_pid_derivative;			// saved locally in memory for convinient debugging. Can be removed to free up memory usage and reduce loading and storing operations
		
		double target_speed;
		
		if(cruise_emergency_stop == 0) {	// check if cruise control emergency stop triggered (breaks used)
			
			read_AN2();			// find the user input as target speep in cruise control (stored in "dash_motor_pot")
			
			target_speed = dash_motor_pot * 0.029326;		// 30KMPH is the maximum allowed speed to cruise / 1023 is the maximum value of the A/D -> 30/1023 = 0.029326
			
			// check if user changed the cruise speed he wants to achieve
			if(target_speed > previous_dynamic_target_speed + target_speed_reading_error_margin || target_speed < previous_dynamic_target_speed - target_speed_reading_error_margin) { // the margin is to minimize A/D error
				total_pid_integration = 0;
				previous_dynamic_target_speed = target_speed;
			}
			
			total_pid_proportional = K * (target_speed - cur_KMPH);
			total_pid_integration += (K * (target_speed - cur_KMPH)) / samples_timeout_in_s;
			total_pid_derivative = (K * ((target_speed - cur_KMPH) - (target_speed - prv_KMPH))) / samples_timeout_in_s;
			
			pid = total_pid_proportional;										// adding the P component of the PID
			pid += total_pid_integration;										// adding the I component of the PID
			pid += total_pid_derivative;										// adding the D component of the PID
			
			// Process PID output into PWM value between [0-199] considering we cap pid to [0-30]
			if(pid >= 30) {
				normalize_duty_cycle(100);	// max power
			} else if(pid <= 0) {
				normalize_duty_cycle(255);	// break
			} else {
				normalize_duty_cycle(pid * 3.333333);	// * (100/30) = * 3.333
			}
		} else if(!PORTDbits.RD3 && !PORTCbits.RC4 && cruise_emergency_stop == 1) {
			normalize_duty_cycle(0);
			cruise_emergency_stop = 2;
		}
	}
	total_sampling_time_elapsed = 0;
}

void read_AN0(void) {
	
	ADCON0bits.ADON = 0b1;				// analog to digital converter ON to find potentiometer value
	ADCON0bits.CHS = 0b0000;			// set A/D to channel 0 (i.e. AN0)		
	ADCON0bits.GO_NOT_DONE = 0b1;		// analog to digital converter in progress
	while(ADCON0bits.GO_NOT_DONE){ }	// wait till the conversion is done
	gear_position = ADRESH;
	gear_position = gear_position << 8;
	gear_position += ADRESL;		// assign output by assigning higher nibble, shifting 8 bits and adding lower nibble
	ADRESH = 0;
	ADRESL = 0;
	ADCON0bits.ADON = 0b0;				// analog to digital converter OFF when finished
	
}

void read_AN1(void) {
	
	ADCON0bits.ADON = 0b1;				// analog to digital converter ON to find potentiometer value
	ADCON0bits.CHS = 0b0001;			// set A/D to channel 1 (i.e. AN1)		
	ADCON0bits.GO_NOT_DONE = 0b1;		// analog to digital converter in progress
	while(ADCON0bits.GO_NOT_DONE){ }	// wait till the conversion is done
	dash_gear_pot = ADRESH;
	dash_gear_pot = dash_gear_pot << 8;
	dash_gear_pot += ADRESL;		// assign output by assigning higher nibble, shifting 8 bits and adding lower nibble
	ADRESH = 0;
	ADRESL = 0;
	ADCON0bits.ADON = 0b0;				// analog to digital converter OFF when finished
	
}

void read_AN2(void) {
	
	ADCON0bits.ADON = 0b1;				// analog to digital converter ON to find potentiometer value
	ADCON0bits.CHS = 0b0010;			// set A/D to channel 2 (i.e. AN2)		
	ADCON0bits.GO_NOT_DONE = 0b1;		// analog to digital converter in progress
	while(ADCON0bits.GO_NOT_DONE){ }	// wait till the conversion is done
	dash_motor_pot = ADRESH;
	dash_motor_pot = dash_motor_pot << 8;
	dash_motor_pot += ADRESL;		// assign output by assigning higher nibble, shifting 8 bits and adding lower nibble
	ADRESH = 0;
	ADRESL = 0;
	ADCON0bits.ADON = 0b0;				// analog to digital converter OFF when finished
	
}

unsigned int calculate_automated_gear(void) {
	
	if(cur_KMPH < gear_1_thresh_KMPH) {
		return gear_1;
	}else if(cur_KMPH >= gear_1_thresh_KMPH && cur_KMPH < gear_2_thresh_KMPH && cadence < cadence_threshold) {
		return gear_1;
	} else if(cur_KMPH >= gear_2_thresh_KMPH && cur_KMPH < gear_3_thresh_KMPH && cadence >= cadence_threshold) {
		return gear_2;
	} else if(cur_KMPH >= gear_3_thresh_KMPH && cur_KMPH < gear_4_thresh_KMPH && cadence < cadence_threshold) {
		return gear_2;
	} else if(cur_KMPH >= gear_3_thresh_KMPH && cur_KMPH < gear_4_thresh_KMPH && cadence >= cadence_threshold) {
		return gear_3;
	} else if(cur_KMPH >= gear_4_thresh_KMPH && cur_KMPH < gear_5_thresh_KMPH && cadence < cadence_threshold) {
		return gear_3;
	} else if(cur_KMPH >= gear_4_thresh_KMPH && cur_KMPH < gear_5_thresh_KMPH && cadence >= cadence_threshold) {
		return gear_4;
	} else if(cur_KMPH >= gear_5_thresh_KMPH) {
		return gear_5;
	} 
	
	return 0;
	
}


void update_gear_stepper(unsigned int target_gear) {	// target_gear[0, 1023] | steps (i.e. if 200, each update results in 180deg shaft rotation)
	
	//Find-out if currently pedaling
	if(cadence != 0) {
		unsigned char i;
		
		read_AN0();    // gear_position
		
		if(gear_position < (target_gear - gear_error_margin)) {
			gear_stepper_step(0);
			/*i = 0;
			while(i < gear_stepper_step_size) { 
				gear_stepper_step(0);
				i = i + 1;
			}*/
			
		}
		
		else if(gear_position > (target_gear + gear_error_margin)) {
			gear_stepper_step(1);
			/*i = 0;
			while(i < gear_stepper_step_size) { 
				gear_stepper_step(1);
				i = i + 1;
			}*/
			
		}
	}
}

void update_speedometer_stepper(void) {
	
	//unsigned char quantized_speed_step = cur_KMPH;	// convert from floating point to integer type in order to floor the result into the lower bound integer value
	signed char delta;
	
	/////
	unsigned char quantized_speed_step;
	if(PORTAbits.RA3) {
		read_AN0();
		quantized_speed_step = 30 - (gear_position * 0.02933);
	} else if(PORTAbits.RA4) {
		read_AN1();
		quantized_speed_step = 30 - (dash_gear_pot * 0.02933);
	} else {
		read_AN2();
		quantized_speed_step = 30 - (dash_motor_pot * 0.02933);
	}
	/////////
	
	if(current_speedometer_step == -1) {	// check if speedometer is out of service
		return;
	}
	
	if(quantized_speed_step > 30) {		// if speed is greater than 30KMPH then it is considered to be 30KMPH on the speedometer
		quantized_speed_step = 30;
	}
	
	delta = quantized_speed_step - current_speedometer_step;	// 1KMPH = 1deg
	
	if(delta > 0) {
		speedometer_stepper_step(0);
		/*unsigned char i = 0;
		while(i < delta){
			speedometer_stepper_step(0);
			i = i + 1;
		}*/
	} else if(delta < 0) {
		speedometer_stepper_step(1);
		/*unsigned char i = 0;
		while(i < -delta){
			speedometer_stepper_step(1);
			i = i + 1;
		}*/
	}
	
	current_speedometer_step = quantized_speed_step;
	
}

void calibrate_speedometer_stepper(signed char limit) {
	while(limit >= 0) {
						
		if(PORTDbits.RD2) {		// speedometer stopper spring reached?
			
			unsigned char count = 0;
			
			while(count < speedometer_calibration_recheck_steps) {	
				speedometer_stepper_step(0);		// step away from the spring and go back to make sure we did not go in the negative direction more than one step
				count = count + 1;
			}
			
			while(!PORTDbits.RD2 && count > 0) {
				speedometer_stepper_step(1);
				count = count - 1;
			}
			
			current_speedometer_step = 0;
			return;
		} 
		speedometer_stepper_step(1);
		limit = limit - 1;
	}
	current_speedometer_step = -1;		// speedometer is out of service
}

void gear_stepper_step(unsigned char direction) {	//200steps = 180deg -> D6 chooses the stepper driver's direction, and D4 the step intialization
	LATDbits.LATD6 = direction;
	
	LATDbits.LATD4 = 0b1;	
	Delay10TCYx(100);
	LATDbits.LATD4 = 0b0;	
	Delay10TCYx(100);
}	

void speedometer_stepper_step(unsigned char direction) {	//12steps = 360deg -> turn on every other coil of the stepper in the below patterns to turn the shaft of the motor
	
	unsigned char d = 25; // 25

	if(direction) {
		if(LATAbits.LATA5 && !LATEbits.LATE0 && !LATEbits.LATE1 && LATEbits.LATE2) {
			LATAbits.LATA5 = 0b1;	
			LATEbits.LATE0 = 0b0;	
			LATEbits.LATE1 = 0b1;	
			LATEbits.LATE2 = 0b0;
				
			Delay10KTCYx(d);
		} else if(LATAbits.LATA5 && !LATEbits.LATE0 && LATEbits.LATE1 && !LATEbits.LATE2) {
			LATAbits.LATA5 = 0b0;	
			LATEbits.LATE0 = 0b1;	
			LATEbits.LATE1 = 0b1;	
			LATEbits.LATE2 = 0b0;
	
			Delay10KTCYx(d);
		} else if(!LATAbits.LATA5 && LATEbits.LATE0 && LATEbits.LATE1 && !LATEbits.LATE2) {
			LATAbits.LATA5 = 0b0;	
			LATEbits.LATE0 = 0b1;	
			LATEbits.LATE1 = 0b0;	
			LATEbits.LATE2 = 0b1;
	
			Delay10KTCYx(d);
		} else if(!LATAbits.LATA5 && LATEbits.LATE0 && !LATEbits.LATE1 && LATEbits.LATE2) {
			LATAbits.LATA5 = 0b1;	
			LATEbits.LATE0 = 0b0;	
			LATEbits.LATE1 = 0b0;	
			LATEbits.LATE2 = 0b1;
			
			Delay10KTCYx(d);
		} else {
			LATAbits.LATA5 = 0b1;	
			LATEbits.LATE0 = 0b0;	
			LATEbits.LATE1 = 0b1;	
			LATEbits.LATE2 = 0b0;
				
			Delay10KTCYx(d);
		}
	} else {
		if(LATAbits.LATA5 && !LATEbits.LATE0 && !LATEbits.LATE1 && LATEbits.LATE2) {
			LATAbits.LATA5 = 0b0;	
			LATEbits.LATE0 = 0b1;	
			LATEbits.LATE1 = 0b0;	
			LATEbits.LATE2 = 0b1;
				
			Delay10KTCYx(d);
		} else if(!LATAbits.LATA5 && LATEbits.LATE0 && !LATEbits.LATE1 && LATEbits.LATE2) {
			LATAbits.LATA5 = 0b0;	
			LATEbits.LATE0 = 0b1;	
			LATEbits.LATE1 = 0b1;	
			LATEbits.LATE2 = 0b0;
	
			Delay10KTCYx(d);
		} else if(!LATAbits.LATA5 && LATEbits.LATE0 && LATEbits.LATE1 && !LATEbits.LATE2) {
			LATAbits.LATA5 = 0b1;	
			LATEbits.LATE0 = 0b0;	
			LATEbits.LATE1 = 0b1;	
			LATEbits.LATE2 = 0b0;
	
			Delay10KTCYx(d);
		} else if(LATAbits.LATA5 && !LATEbits.LATE0 && LATEbits.LATE1 && !LATEbits.LATE2) {
			LATAbits.LATA5 = 0b1;	
			LATEbits.LATE0 = 0b0;	
			LATEbits.LATE1 = 0b0;	
			LATEbits.LATE2 = 0b1;
			
			Delay10KTCYx(d);
		} else {
			LATAbits.LATA5 = 0b0;	
			LATEbits.LATE0 = 0b1;	
			LATEbits.LATE1 = 0b0;	
			LATEbits.LATE2 = 0b1;
				
			Delay10KTCYx(d);
		}
	}
}