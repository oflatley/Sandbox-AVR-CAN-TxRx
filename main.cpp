/*
 * MyAVRCAN.cpp
 *
 * Created: 6/1/2020 1:34:48 PM
 * Author : oflat
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>

#include <AvrCAN.h>


#include "types.h"



static const u16 OmegaBaseArbitrarionId = 0x100;
static const u16 KphAndRpmArbitrarionId = (OmegaBaseArbitrarionId + 0x70);
static const u16 EngineCelsiusArbitrationId = (OmegaBaseArbitrarionId + 0x10);
static const u16 AlarmsDashLightFlagsArbitrarionId = (OmegaBaseArbitrarionId + 0xA8);
static const u16 OilPressureBarArbitrarionId = (OmegaBaseArbitrarionId + 0x28);
static const u16 EngineErrorFlagsArbitrarionId = (OmegaBaseArbitrarionId + 0x78);
static const u16 FuelLevelCustomStreamArbitrationId = 0x200;       // From custom-stream.   we expect 1 word.  between 0..{n<=0x300}  ==> empty..full




volatile uint8_t CTC_flag = 0;

ISR(TIMER1_COMPA_vect) {
	CTC_flag = 1;
}

// This function initializes a 16-bit timer used for delays.
void timer_init(void) {
	
	TIMSK1 = 0x00; //Disable timer interrupts
	TCCR1A = 0x00; //Timer not connected to any pins
	TCCR1B = 0x0A; //CTC mode; Timer_Rate = System_CLK/8 = 1MHz
	// 1 tick = 1 us (assume system clock = 8MHz)
}

// This is a blocking function that sets the timer value
// and waits until the value is reached. Waits for interrupt
// to execute.  Parameter "us" is truly 1 micro-second only
// if timer rate is set to 1MHz frequency. For example, this
// project uses a system clock of 8MHz. Timer1 rate is set to
// System Clock divided by 8 which is 1MHz (in timer_init).
// Therefore the "us" parameter truly is 1us.
void delay_us(uint16_t us) {
	CTC_flag = 0; //Clear CTC Interrupt flag
	TCNT1 = 0x0000; //Clear timer
	OCR1A = us; //Load compare value with desired delay in us
	TIMSK1 = 0x02; //Enable OCIE1A Interrupt
	while(!CTC_flag); //wait for the designated number of us
}

void delay_ms(uint16_t ms) {
	for(uint16_t i=0; i<ms; i++) {
		delay_us(1000);
	}
}







void LED_Reg_Write(uint8_t val) 
{
	PORTB = ~val;
}



// buttonIndex [0..7]
u8 IsButtonPressed(u8 buttonIndex);



u8 readButton( u8 index )
{
	// flipped for Stk600
	return !(PIND & (1 << (index & 0xF) ));
}



// Read button with crude debounce
uint8_t IsButtonPressed (u8 buttonIndex) {
	// button debounce
	if(readButton(buttonIndex))
	{
		
		delay_ms(50);
		
		while(readButton(buttonIndex))
		{			
		}

		delay_ms(50);
		
		return 1;

	}
	return 0;
}



void io_init(void) {
	
	// Init PORTB[7:0] // LED port
	DDRB = 0xFF;
	LED_Reg_Write(0x00); // clear all LEDs
	
	// Init PORTC[7:0]
	// PORTC[3:2] => RXCAN:TXCAN

	DDRC = 0x00;
	
	
	// Init PORTD[7:0]
	// PORTD[7:0] => Debug Trigger Switches
	DDRD = 0x00;
	// activate pull-ups on D (config'd as Input).
	PORTD = 0xFF;
	
	
	// Init PORTE[2:0] // not used
	DDRE = 0x00;
	PORTE = 0x00;
}


static volatile u8 errorCount = 0;
static volatile u8 txCount = 0;
static volatile u8 rxCount = 0;


volatile u8 vMI = 11;
volatile u16 vAId = 0;
volatile u8 vDlc = 0;
u8 Data[8];




static u8* GetMessage()
{
	
	static u8 data[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	
	if( !++data[0] && !++data[1] && !++data[2] && !++data[3] && !++data[4] && !++data[5] && !++data[6] && ++data[7] ) {}

	data[5] = txCount;
	data[6] = rxCount;
	data[7] = errorCount;

	return data;	
}



volatile u16 wtf;
volatile u16 MIL = 0;
volatile u16 OilP = 0;
volatile u16 RPM = 0;
volatile u16 Gas = 0;
volatile u16 Temp = 0;
volatile u16 MPH = 0;
volatile u16 OilBar = 0;

static void OnTXComplete( u16 arbitrationId, u8 dataLength, u8* data )
{

	txCount++;

	for( u8 n= 0; n < dataLength; ++n )
	{
		Data[n] = data[n];
	}
	
	vAId = arbitrationId;
	vDlc = dataLength;	
}

static void OnError( u8 flags )
{
	errorCount++;	
}





static void ReceivedKpmAndRPM( u16 arbitrationId, u8 dataLength, u8* data)
{

	for( u8 n= 0; n < dataLength; ++n )
	{
		Data[n] = data[n];
	}
	
	vAId = arbitrationId;
	vDlc = dataLength;

	RPM++;
	MPH++;	
}

static void ReceivedTempCelsius( u16 arbitrationId, u8 dataLength, u8* data)
{
	for( u8 n= 0; n < dataLength; ++n )
	{
		Data[n] = data[n];
	}
	
	vAId = arbitrationId;
	vDlc = dataLength;

	Temp++;
	
}

static void ReceivedOilPBar( u16 arbitrationId, u8 dataLength, u8* data)
{
	for( u8 n= 0; n < dataLength; ++n )
	{
		Data[n] = data[n];
	}
	
	vAId = arbitrationId;
	vDlc = dataLength;

	OilBar++;
	
}


static void ReceivedFuelLevelScaler( u16 arbitrationId, u8 dataLength, u8* data)
{
	for( u8 n= 0; n < dataLength; ++n )
	{
		Data[n] = data[n];
	}
	
	vAId = arbitrationId;
	vDlc = dataLength;

	Gas++;
	
}

static void ReceivedFlags( u16 arbitrationId, u8 dataLength, u8* data)
{
	switch(arbitrationId)
	{
		case AlarmsDashLightFlagsArbitrarionId:
		MIL++;
		break;
		case EngineErrorFlagsArbitrarionId:
		OilP++;
		break;
		default:
		wtf = arbitrationId;
		break;
	}

	
}









int main(void)
{
	io_init();
	timer_init();
	sei();


	
	u8 dataRX[8];


	// lets try individual boxes for all BUT the warnings.  those will get caught in a everything-else box








	// highest priority
	AvrCAN::AddInbox( KphAndRpmArbitrarionId, 0xFFFF, 8, dataRX, ReceivedKpmAndRPM );	
	AvrCAN::AddInbox( EngineCelsiusArbitrationId, 0xFFFF, 8, dataRX, ReceivedTempCelsius  );	
	AvrCAN::AddInbox( OilPressureBarArbitrarionId, 0xFFFF, 8, dataRX, ReceivedOilPBar );	
	AvrCAN::AddInbox( FuelLevelCustomStreamArbitrationId, 0xFFFF, 8, dataRX, ReceivedFuelLevelScaler );	

	// and the rest ...
	// AlarmsDashLightFlagsArbitrarionId
	// EngineErrorFlagsArbitrarionId
	AvrCAN::AddInbox( 0, 0, 8, dataRX, ReceivedFlags );	// all other arbitrationId
	// lowest priority		
		
														// for example
	//AvrCAN::AddInbox( (u16)1, (u16)1, 8, dataRX, ReceivedRXOdd );	// odd arbitrationId
	//AvrCAN::AddInbox( (u16)0, (u16)1, 8, dataRX, ReceivedRXEven );	// even arbitrationId	
	
	// clears, enables, sets interrupts, then walks Added MessageBoxs and sets ids and filters
	AvrCAN::Start( OnTXComplete, OnError );

	
	// Flash LEDs to indicate program startup
	for( u8 n = 0; n <8 ; n++ )
	{		
		LED_Reg_Write(_BV(n));
		delay_ms(25);
	}

	
	LED_Reg_Write(0x00);

	
	while(1)
	{
		if(IsButtonPressed(0))
		{
			AvrCAN::SendMessage( 0x51, 8, GetMessage() );
		}
	}
}

