#include <Arduino.h>
#include <LiquidCrystal.h>

#define ocr_20ms_ctc 4999
#define RIGHT 0
#define UP 1
#define DOWN 2
#define LEFT 3
#define SELECT 4
#define NO_BUTTON 5

#define RIGHT_VAL 0
#define UP_VAL 1
#define DOWN_VAL 2
#define LEFT_VAL 4
#define SELECT_VAL 6

#define MAIN 0
#define TSET 1
#define TINC 2
#define TMENT 3
#define TRAC 4
#define KP 5
#define KI 6
#define KD 7

#define TEMP_CAMERA 30

#define INCALZIRE 0
#define MENTINERE 1
#define RACIRE 2

#define KP_LOC 0
#define KI_LOC 4
#define KD_LOC 8
#define TINC_LOC 12
#define TMEN_LOC 13
#define TRAC_LOC 14
#define TSET_LOC 15
#define INIT_FLAG 16

#define INIT_CONST 'I'

LiquidCrystal lcd( 8, 9, 4, 5, 6, 7 );

volatile double kp,ki,kd,err=0,err_prec=0,I=0,D=0,Ts=0.02,temp=0;
volatile uint8_t sec=0,m=0,h=0,temp_set,tinc,trac,tmen,phase=0,setpoint;
volatile unsigned long cnt=0,cnt_begin_phase=0;
volatile uint8_t buton_prev=NO_BUTTON,buton=NO_BUTTON,state=MAIN;

void timer1_ctc_20ms(){
  TCCR1B=(1<<WGM12)|(1<<CS11)|(1<<CS10);
  OCR1A=ocr_20ms_ctc;
  TIMSK1=(1<<OCIE1A);
}

void set_pwm2_width(uint8_t fill){
  uint16_t ocr;
  ocr=(fill*255)/100;
  OCR2A=ocr;
}

void timer2_pwm(){
  TCCR2A=(1<<COM2A1)|(1<<WGM21)|(1<<WGM20);
  TCCR2B=(1<<CS21)|(1<<CS22)|(1<<CS20);
  OCR2A=0;
}

void adc_init()
{
  ADCSRA |= ((1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0));
  ADCSRA |= (1 << ADEN); //enable ADC
  ADMUX |= (1 << REFS0);
  ADCSRA |= (1 << ADSC); //ADC start conversion
}

uint16_t read_adc(int channel)
{
  DDRC &= ~(1 << channel);
  ADMUX &= ~7;
  ADMUX |= (1<<channel);
  ADCSRA |= (1 << ADSC); //start conversion
  while (ADCSRA & (1 << ADSC)); //wait while adc conversion are not updated
  return ADCW; //read and return
}

void regulator(){
  double out,I_temp=I;
  temp=read_adc(2)*0.4882;
  err=setpoint-temp;
  I=I+err*Ts;
  D=(err-err_prec)/Ts;
  out=kp*err+ki*I+kd*D;
  if(out>100) {
    out=100;
  }
  else if(out<0){
    out=0;
    I=0;
  }
  set_pwm2_width(out);
  err_prec=err;
}

void detectie_buton(){
  uint16_t val=read_adc(0)/100;
  buton_prev=buton;
  switch(val){
    case RIGHT_VAL: buton=RIGHT; break;
    case UP_VAL: buton=UP; break;
    case DOWN_VAL: buton=DOWN; break;
    case LEFT_VAL: buton=LEFT; break;
    case SELECT_VAL: buton=SELECT; break;
    default: buton=NO_BUTTON; break;
  }
}

uint8_t buton_apasat(){
  if(buton==buton_prev)
  return NO_BUTTON;
  return buton;
}

void mod_main(){
  lcd.setCursor(0,1);
  lcd.print((uint8_t)temp);
  lcd.print("C|");
  lcd.print(setpoint);
  lcd.print("C; ");
  switch(phase){
    case INCALZIRE: lcd.print("Inc:");
    lcd.print((uint8_t)(tinc-(cnt-cnt_begin_phase)*Ts));
    break;
    case MENTINERE: lcd.print("Men:");
    lcd.print((uint8_t)(tmen-(cnt-cnt_begin_phase)*Ts));
    break;
    case RACIRE: lcd.print("Rac:");
    lcd.print((uint8_t)(trac-(cnt-cnt_begin_phase)*Ts));
    break;
  }
  lcd.print("s ");
}

void mod_tset(){
  static uint8_t tset_temp=temp_set;
  lcd.setCursor(0,1);
  lcd.print("Tset:");
  lcd.print(temp_set);
  lcd.print("C|");
  lcd.print(tset_temp);
  lcd.print("C      ");
  switch(buton_apasat()){
    case UP: tset_temp++;
    break;
    case DOWN: tset_temp--;
    break;
    case SELECT:
      temp_set=tset_temp;
      eeprom_write_byte((uint8_t*)TSET_LOC,temp_set);
      lcd.setCursor(0,1);
      lcd.print("Salvat           ");
      _delay_ms(500);
    break;
    case LEFT: tset_temp=temp_set;
    state=MAIN;
    break;
  }
}

void mod_tincalz(){
  static uint8_t tinc_temp=tinc;
  lcd.setCursor(0,1);
  lcd.print("Tincalz:");
  lcd.print(tinc);
  lcd.print("s|");
  lcd.print(tinc_temp);
  lcd.print("s      ");
  switch(buton_apasat()){
    case UP: tinc_temp++;
    break;
    case DOWN: tinc_temp--;
    break;
    case SELECT: {
      tinc=tinc_temp;
      eeprom_write_byte((uint8_t*)TINC_LOC,tinc);
      lcd.setCursor(0,1);
      lcd.print("Salvat           ");
      _delay_ms(500);
    }
    break;
    case LEFT: tinc_temp=tinc;
    state=MAIN;
    break;
  }
}

void mod_tment(){
  static uint8_t tmen_temp=tmen;
  lcd.setCursor(0,1);
  lcd.print("Tment:");
  lcd.print(tmen);
  lcd.print("s|");
  lcd.print(tmen_temp);
  lcd.print("s   ");
  switch(buton_apasat()){
    case UP: tmen_temp++;
    break;
    case DOWN: tmen_temp--;
    break;
    case SELECT: {
      tmen=tmen_temp;
      eeprom_write_byte((uint8_t*)TMEN_LOC,tmen);
      lcd.setCursor(0,1);
      lcd.print("Salvat           ");
      _delay_ms(500);
    }
    break;
    case LEFT: tmen_temp=tmen;
    state=MAIN;
    break;
  }
}

void mod_tracire(){
  static uint8_t trac_temp=trac;
  lcd.setCursor(0,1);
  lcd.print("Tracire: ");
  lcd.print(trac);
  lcd.print("s|");
  lcd.print(trac_temp);
  lcd.print("s   ");
  switch(buton_apasat()){
    case UP: trac_temp++;
    break;
    case DOWN: trac_temp--;
    break;
    case SELECT: {
      trac=trac_temp;
      eeprom_write_byte((uint8_t*)TRAC_LOC,trac);
      lcd.setCursor(0,1);
      lcd.print("Salvat           ");
      _delay_ms(500);
    }
    break;
    case LEFT: trac_temp=trac;
    state=MAIN;
    break;
  }
}

void mod_kp(){
  static double kp_temp=kp;
  lcd.setCursor(0,1);
  lcd.print("KP: ");
  lcd.print(kp);
  lcd.print("|");
  lcd.print(kp_temp);
  lcd.print("        ");
  switch(buton_apasat()){
    case UP: kp_temp+=0.1;
    break;
    case DOWN:kp_temp-=0.1;
    break;
    case SELECT: {
      kp=kp_temp;
      eeprom_write_float((float*)KP_LOC,kp);
      lcd.setCursor(0,1);
      lcd.print("Salvat           ");
      _delay_ms(500);
    }
    break;
    case LEFT: kp_temp=kp;
    state=MAIN;
    break;
  }
}

void mod_ki(){
  static double ki_temp=ki;
  lcd.setCursor(0,1);
  lcd.print("KI: ");
  lcd.print(ki);
  lcd.print("|");
  lcd.print(ki_temp);
  lcd.print("        ");
  switch(buton_apasat()){
    case UP: ki_temp+=0.1;
    break;
    case DOWN: ki_temp-=0.1;
    break;
    case SELECT: {
      ki=ki_temp;
      eeprom_write_float((float*)KI_LOC,ki);
      lcd.setCursor(0,1);
      lcd.print("Salvat           ");
      _delay_ms(500);
    }
    break;
    case LEFT: ki_temp=ki;
    state=MAIN;
    break;
  }
}

void mod_kd(){
  static double kd_temp=kd;
  lcd.setCursor(0,1);
  lcd.print("KD: ");
  lcd.print(kd);
  lcd.print("|");
  lcd.print(kd_temp);
  lcd.print("        ");
  switch(buton_apasat()){
    case UP: kd_temp+=0.1;
    break;
    case DOWN: kd_temp-=0.1;
    break;
    case SELECT: {
      kd=kd_temp;
      eeprom_write_float((float*)KD_LOC,kd);
      lcd.setCursor(0,1);
      lcd.print("Salvat           ");
      _delay_ms(500);
    }
    break;
    case LEFT: kd_temp=kd;
    state=MAIN;
    break;
  }
}

void eeprom_initialize(){
  eeprom_write_float((float*)KP_LOC,5);
  eeprom_write_float((float*)KI_LOC,0.5);
  eeprom_write_float((float*)KD_LOC,0);
  eeprom_write_byte((uint8_t*)TINC_LOC,30);
  eeprom_write_byte((uint8_t*)TRAC_LOC,50);
  eeprom_write_byte((uint8_t*)TMEN_LOC,50);
  eeprom_write_byte((uint8_t*)TSET_LOC,37);
  eeprom_write_byte((uint8_t*)INIT_FLAG,INIT_CONST);
}

void init_from_eeprom(){
  kp=eeprom_read_float((float*)KP_LOC);
  ki=eeprom_read_float((float*)KI_LOC);
  kd=eeprom_read_float((float*)KD_LOC);
  tinc=eeprom_read_byte((uint8_t*)TINC_LOC);
  trac=eeprom_read_byte((uint8_t*)TRAC_LOC);
  tmen=eeprom_read_byte((uint8_t*)TMEN_LOC);
  temp_set=eeprom_read_byte((uint8_t*)TSET_LOC);
  setpoint=temp_set;
}

void (*modes[8])(void)={mod_main,mod_tset,mod_tincalz,mod_tment,mod_tracire,mod_kp,mod_ki,mod_kd};  //function pointer array

int main(){
  cli();
  if(eeprom_read_byte((uint8_t*)INIT_FLAG)!=INIT_CONST)
    eeprom_initialize();
  init_from_eeprom();
  DDRB|=(1<<3);
  adc_init();
  timer1_ctc_20ms();
  timer2_pwm();
  lcd.begin(16,2);
  sei();
  Serial.begin(9600);
  while(1){
    lcd.setCursor(0,0);
    lcd.print(h);
    lcd.print(":");
    lcd.print(m);
    lcd.print(":");
    lcd.print(sec);
    lcd.print("   ");
    modes[state]();
    if(buton_apasat()==RIGHT)
    if(++state>KD){
      state=MAIN;
      buton_prev=buton;
    }
    Serial.print("Temperatura:");
    Serial.print(temp);
    Serial.print(",");
    Serial.print("Tinta:");
    Serial.println(setpoint);
    _delay_ms(5);
  }
}

ISR(TIMER1_COMPA_vect){
  regulator();
  if(++cnt%50==0){
    unsigned long diff=(cnt-cnt_begin_phase)*Ts;
    if(++sec>=60){
      sec=0;
      m++;
    }
    if(m>=60){
      m=0;
      h++;
    }
    switch(phase){
      case INCALZIRE:if(diff==tinc){
        phase=MENTINERE;
        cnt_begin_phase=cnt;
      }
      break;
      case MENTINERE: if(diff==tmen){
        phase=RACIRE;
        cnt_begin_phase=cnt;
        setpoint=TEMP_CAMERA;
      }
      break;
      case RACIRE: if(diff==trac){
        phase=INCALZIRE;
        cnt_begin_phase=cnt;
        setpoint=temp_set;
      }
      break;
    }
  }
  detectie_buton();
}