#include <LiquidCrystal.h>

volatile int h=12,min=5,s=0;	//ore, minute, secunde - initial
volatile long int temp=0;

LiquidCrystal LCD(12, 11, 5, 4, 3, 2);

void timer1_setup_1s(){
	TCCR1A=0;
	TCCR1B=0X04|(1<<WGM12);
	TIMSK1|=(1<<OCIE1A);
	OCR1A=62500;
}

void adc_init()
{
  ADCSRA |= ((1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0));
  ADCSRA |= (1 << ADEN); //enable ADC
  ADMUX |= (1 << REFS0);
  ADCSRA |= (1 << ADSC); //ADC start conversion
}

int read_adc(int channel)
{
  DDRC &= ~(1 << channel);
  ADMUX &= ~7;
  ADMUX |= channel;//select chanel AO to A5
  ADCSRA |= (1 << ADSC); //start conversion
  while (ADCSRA & (1 << ADSC)); //wait while adc conversion are not updated
  return ADCW; //read and return
}

void setup(){
  cli();
  timer1_setup_1s();
  adc_init();
  LCD.begin(16,2);
  sei();
}

void loop(){
  LCD.setCursor(0,0);
  LCD.print("Temp: ");
  LCD.print(temp);
  LCD.print("C       ");
  LCD.setCursor(0,1);
  LCD.print("Ora: ");
  LCD.print(h);
  LCD.print(":");
  LCD.print(min);
  LCD.print(":");
  LCD.print(s);
  LCD.print("   ");
}

ISR(TIMER1_COMPA_vect){
  sei();
  temp=read_adc(0);
  temp=(temp*488)/100-500;
  temp=temp/10;
  s++;
  if(s==60)
    min++;
  if(min==60)
    h++;
  s=s%60;
  min=min%60;
  h=h%24;
}