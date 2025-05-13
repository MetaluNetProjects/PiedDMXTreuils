/*********************************************************************
 *               Derby2 Cheval
 * .board: FraiseStep 1.2
 *  Antoine Rousseau @ metalu.net - jan.2022
 *********************************************************************/

#define BOARD Step1.2

#include <fruit.h>
#include <analog.h>
#include <dcmotor.h>
#include <ramp.h>
#include <eeparams.h>
//#include <servo.h>


//t_ramp posRamp;
t_delay mainDelay;

#ifdef _DCMOTOR_H_
#define USE_MOTORS
#endif

#ifdef USE_MOTORS
DCMOTOR_DECLARE(A);
DCMOTOR_DECLARE(B);
#endif


//-------------  Timer1 macros :  ---------------------------------------- 
//prescaler=PS fTMR1=FOSC/(4*PS) nbCycles=0xffff-TMR1init T=nbCycles/fTMR1=(0xffff-TMR1init)*4PS/FOSC
//TMR1init=0xffff-(T*FOSC/4PS) ; max=65536*4PS/FOSC : 
//ex: PS=8 : T=0.01s : TMR1init=0xffff-15000
//Maximum 1s !!
#define	TMR1init(T) (0xffff-((T*FOSC)/32000)) //ms ; maximum: 8MHz:262ms 48MHz:43ms 64MHz:32ms
#define	TMR1initUS(T) (0xffff-((T*FOSC)/32000000)) //us ; 
#define InitTimer(T) do{ TMR1H=TMR1init(T)/256 ; TMR1L=TMR1init(T)%256; PIR1bits.TMR1IF=0; }while(0)
#define InitTimerUS(T) do{ TMR1H=TMR1initUS(T)/256 ; TMR1L=TMR1initUS(T)%256; PIR1bits.TMR1IF=0; }while(0)
#define TimerOut() (PIR1bits.TMR1IF)

void highInterrupts()
{
	if(PIR1bits.TMR1IF) {
		DCMOTOR_CAPTURE_SERVICE(A);
		InitTimerUS(50UL);
	}
	//servoHighInterrupt();
}

void setup(void) {	
//----------- Setup ----------------
	fruitInit();
			
	pinModeDigitalOut(LED); 	// set the LED pin mode to digital out
	digitalClear(LED);		// clear the LED
	delayStart(mainDelay, 5000); 	// init the mainDelay to 5 ms

//----------- Analog setup ----------------
	analogInit();		// init analog module
//	analogSelect(0, MASENSE);	// assign MotorA current sense to analog channel 0

//----------- Servo setup ----------------
	/*servoInit();        // init servo module
	servoSelect(0,SERVO1); // */

//----------- dcmotor setup ----------------

#ifdef USE_MOTORS
	dcmotorInit(A);
//	dcmotorInit(B);
//	pinModeDigitalIn(TRANS_LOSW);
//	pinModeDigitalIn(TRANS_HISW);

// power leds
	/*digitalSet(MBEN);
	pinModeDigitalOut(MBEN);
	
	pinModeAnalogOut(PWLED1);
	pinModeAnalogOut(PWLED2);
	analogWrite(PWLED1, 0);
	analogWrite(PWLED2, 0);*/
#endif
	

	DCMOTOR(A).Setting.onlyPositive = 1;
	DCMOTOR(A).Setting.PosWindow = 10;
	DCMOTOR(A).Setting.PwmMin = 100;
	DCMOTOR(A).Setting.PosErrorGain = 6;//9;
//#define HW_PARAMS	
#ifdef HW_PARAMS
	DCMOTOR(A).Setting.PosWindow = 6;
	DCMOTOR(A).Setting.PwmMin = 50;
	//DCMOTOR(D).Setting.PosErrorGain = 6;
	//DCMOTOR(D).Setting.onlyPositive = 0;
	
	DCMOTOR(A).PosRamp.maxSpeed = 1800;
	DCMOTOR(A).PosRamp.maxAccel = 2400;
	DCMOTOR(A).PosRamp.maxDecel = 2400;
	//rampSetPos(&DCMOTOR(C).PosRamp, 0);

	DCMOTOR(A).PosPID.GainP = 40;
	DCMOTOR(A).PosPID.GainI = 1;
	DCMOTOR(A).PosPID.GainD = 0;
	DCMOTOR(A).PosPID.MaxOut = 1023;

	//DCMOTOR(C).VolVars.homed = 0;
#else
	EEreadMain();
#endif
	
//	DCMOTOR(B).Setting.reversed = 1;

	T1CON=0b00110011;//src=fosc/4,ps=8,16bit r/w,on.
	PIE1bits.TMR1IE=1;  //1;
	IPR1bits.TMR1IP=1;
}

void sendMotorState()
{
	static unsigned char buf[20] = { 'B', 10};
	static int ramppos;
	static unsigned len;
	
	len = 2;
	buf[len++] = DCMOTOR_GETPOS(A) >> 24;
	buf[len++] = DCMOTOR_GETPOS(A) >> 16;
	buf[len++] = DCMOTOR_GETPOS(A) >> 8;
	buf[len++] = DCMOTOR_GETPOS(A) & 255;
	buf[len++] = digitalRead(MOTA_END) == MOTA_ENDLEVEL;
	buf[len++] = 0;//digitalRead(TRANS_HISW) == TRANS_SWLEVEL;
	buf[len++] = DCMOTOR(A).Vars.PWMFinal >> 8;
	buf[len++] = DCMOTOR(A).Vars.PWMFinal & 255;
	ramppos = (int)rampGetPos(&(DCMOTOR(A).PosRamp));
	buf[len++] = ramppos >> 24;
	buf[len++] = ramppos >> 16;
	buf[len++] = ramppos >> 8;
	buf[len++] = ramppos & 255;
	buf[len++] = DCMOTOR(A).VolVars.homed;
	buf[len++] = '\n';
	fraiseSend(buf,len);
}

void loop() {
	static unsigned char loopCount;
// ---------- Main loop ------------
	fraiseService();	// listen to Fraise events
	analogService();	// analog management routine
	//servoService();	// servo management routine

	if(delayFinished(mainDelay)) // when mainDelay triggers :
	{
		delayStart(mainDelay, 10000); 	// re-init mainDelay
		//analogSend();		// send analog channels that changed
		//fraiseService();
#ifdef USE_MOTORS
		DCMOTOR_COMPUTE(A, ASYM);
		
		if(loopCount++ > 10) {
			sendMotorState();
			loopCount = 0;
		}
#endif
	}
}

// Receiving

void fraiseReceiveChar() // receive text
{
	unsigned char c;
	
	c=fraiseGetChar();
	if(c=='L'){		//switch LED on/off 
		c=fraiseGetChar();
		digitalWrite(LED, c!='0');		
	}
	else
	if(c=='E') { 	// echo text (send it back to host)
		printf("C");
		c = fraiseGetLen(); 			// get length of current packet
		while(c--) printf("%c",fraiseGetChar());// send each received byte
		putchar('\n');				// end of line
	}
	else if(c=='W') { 	// WRITE: save eeprom
		if((fraiseGetChar() == 'R') && (fraiseGetChar() == 'I') && (fraiseGetChar() == 'T') && (fraiseGetChar() == 'E'))
			EEwriteMain();
	}
}

void fraiseReceive() // receive raw
{
	unsigned char c;
	unsigned int i;
	
	c=fraiseGetChar();
	
	switch(c) {
#ifdef USE_MOTORS
	    case 120 : DCMOTOR_INPUT(A) ; break;
#endif
	}
}

void EEdeclareMain()
{
	DCMOTOR_DECLARE_EE(A);
}
