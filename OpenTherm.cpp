#include "OpenTherm.h"


inline void OpenTherm::receive(){
  cli();
  first=1;//receiving first bit
  buf=0;//clear buffer
  rx=1;
  data_ready=0;
  length=0;
  parity=0;


	PCICR|=bit(rx_pcie); //enabling Pin Change Interrupt for RX port
	*rx_pcmsk|=rx_bitmask; //enabling Pin Change Interrupt for RX pin of rx Port

  
  TCCR2A=bit(WGM21);//mode CTC
  TCCR2B=0;
  OCR2A=TICKS_PER_MS*0.75;//interrupt at 0.75ms
  TCNT2=TICKS_PER_MS*0.5;//preload 0.5ms
  TIMSK2=bit(OCIE2A);//interrupt on OC0A
  TIFR2=bit(OCF2A);
  sei();
}




/*
Using PCINT only for receiving frame.

On start bit, setting interrupt activation on signal change,
starts Timer/Counter2, which work in CTC mode with period
of 0.75ms (3/4 Opentherm period) that cause TIMER2_COMPA 
to fire somwhere in the middle of first half of next period (idealy 
at 1/4 of next period).

On every regular interrupt (right at signal change at the middle of period)
clears Timer/Counter2 for synchronization, and disabling PCINT interrupt for
skipping plausible level change at start of next bit period.
(On PCINT we catch only compulsory level change at the middle of bit period)

On last 33 bit (stop bit) set flag data_ready and disable Timer/Counter2
and it's interrupts
*/
bool OpenTherm::extIntHandler(){
  
  *rx_pcmsk&=~rx_bitmask; //Disable PCINT for RX pin

  if (first) {
    first=0;
  }
  else{
    TCNT2=0;
  }
  
  if (length > MSG_LENGTH){ 
    data_ready=1;
    TCCR2B=0;//disable Timer2
    TIMSK2=0;
    return 1;
  } else TCCR2B=bit(CS22); //start timer clk/64
	return 0;  
};

/*
Interrupt TIMER2_COMPA  used for receiving adn transmitting frames

While transmitting we set transmit pin HIGH or LOW according to
MSB of frame buffer at the start of every bit period (every 1 ms).
Than doing left shift of data buffer and if transmitting first bit,
adding 1 at the end (last stop bit).

While receiving TIMER2_COMPA fires every 0.75ms right at the middle of
first half of bit period. Reading INT0 state (receiving pin) and adding 
its value to the end frame buffer, for bits 2-31 (0-start bit,1- parity) 
calculating parity.
At the end enabling again INT0 interrupt, that should be triggered on compulsory 
level change at the half of the bit period (approx. 0.25ms).


*/

void OpenTherm::timer2CompAHandler(){
  uint32_t tmp_buf;
  tmp_buf=buf;
  if (!rx ) {
   if (tmp_buf&bit(31))
     *(tx_port+2)|=tx_bitmask; //set OUTPUT HIGH
   else
     *(tx_port+2)&=~tx_bitmask; //set OUTPUT LOW
  };
  
  tmp_buf=tmp_buf<<1;
  if (!rx && first) {
    tmp_buf|=1; //adding stop bit for transmitting message
    first=0;
  }

  if (rx) {
    if (*rx_port & rx_bitmask) { //Reading RX value
      tmp_buf=tmp_buf | 1;
      if (length > 1 ) parity^=1;
    }
    *rx_pcmsk|=rx_bitmask; //Re-enabling PCINT for RX pin
    TCCR2B=0;
  }
  length++;
  buf=tmp_buf;
  
   
 
  
};
/*
TIMER2_COMPB used only while transmitting for
compulsory level change at the middle of bit period.
After transmitting whole frame with stop bit
setting flag data_ready and disabling TIMER2 and it's interrupts.
*/
void OpenTherm::timer2CompBHandler(){
 
  *tx_port = tx_bitmask; //Inverting TX pin

  if (length > MSG_LENGTH) {
  receive();
  }

    
};


OpenTherm::OpenTherm(uint8_t rx_pin, uint8_t tx_pin) {
 
  tx_port=portInputRegister(digitalPinToPort(tx_pin));
 	tx_bitmask=digitalPinToBitMask(tx_pin);

 	rx_port=portInputRegister(digitalPinToPort(rx_pin));
 	rx_bitmask=digitalPinToBitMask(rx_pin);
 	
  rx_pcmsk=digitalPinToPCMSK(rx_pin); //PCMSK num for rx pin
  rx_pcie=digitalPinToPCICRbit(rx_pin); // PCIE bit of PCICR for rx pin
  
  *(tx_port+1)|=tx_bitmask; //TX pin output
  *(tx_port+2)&=~tx_bitmask; //LOW level
 
  *(rx_port+1)&=~rx_bitmask; //RX pin input
  *(rx_port+2)&=~rx_bitmask; //No pullup

  state=125;//reading slave version
  CH_enabled=1;
  DHW_enabled=1;
  ts=100;
  DHW_max_lim=60;
  DHW_min_lim=40;
  CH_max_lim=65;
  CH_min_lim=30;

};

void OpenTherm::begin(ot_init_settings* init){
    CH_enabled=init->CH_enabled;
    DHW_enabled=init->DHW_enabled;
    CH_max=init->CH_max_temp;
    target_CH=init->CH_temp;
    target_DHW=init->DHW_temp;
    max_modulation=init->max_modulation;
    
    delay(1000);
    update();//slave OT version req
    delay(1000);
    update();//slave cfg and member_id req
    delay(1000);
    update(57);//Upper CH bound
    delay(1000);
    update(56); //DHW set
    delay(1000);
    update(14); //max modulation set
   
}

void OpenTherm::abort(){
  TCCR2A=0;
  TCCR2B=0;
  TIMSK2=0;
  TCNT2=0;
  OCR2A=0;
  OCR2B=0;
  *rx_pcmsk&=~rx_bitmask;
  *(tx_port+2)&=~tx_bitmask; //LOW level
  data_ready=0;
  rx=0;
  length=0;
  buf=0;
}
float OpenTherm::OT_to_float(uint16_t data){
return (float)data/256.0;
};

uint16_t OpenTherm::float_to_OT(float data){
return (((uint16_t)data)<<8) + (uint8_t)((data-(int)data)*256);
};


void OpenTherm::send(){
  cli();
  rx=0;
  length=0;
  data_ready=0;
  first=1;
  TCCR2A=bit(WGM21);//mode CTC
  OCR2B=TICKS_PER_MS*0.5-1; //0.5ms - toggle out
  OCR2A=TICKS_PER_MS-1; //1ms
  TCNT2=0;
  ASSR=0;
  TIMSK2|=bit(OCIE2A)|bit(OCIE2B);
  TIFR2|=bit(OCF2A)|bit(OCF2B);
  TCCR2B=bit(CS22); //start timer clk/64
  *rx_pcmsk&=~rx_bitmask; //Disabling interrupt on RX while transmitting frame
  TIFR2|=bit(OCF2A)|bit(OCF2B);
	*(tx_port+2)|=tx_bitmask;
  sei();
}

void OpenTherm::communicate(const uint8_t type, const uint8_t id,const uint16_t data){
  uint8_t par=0,i=0;
  uint32_t tmp=0;
  tmp=((uint32_t)type<<28) + ((uint32_t)id<<16) + data;
  for(i=0;i<31;i++) {
    par^=(tmp&bit(i)?1:0);
  }
  tmp|=par?bit(31):0;
  buf=tmp;
  send();
  ts=millis();
}

uint8_t OpenTherm::complete(uint8_t *type,uint8_t *id,uint16_t *data){
  uint32_t tmp=buf,delta=millis()-ts;

		
		
    if( (data_ready && parity!=(buf&bit(31)?1:0)) || (!data_ready && delta>1200) ) {        //received bad frame or time error;
        abort();
        return 2;
    }

		if(!rx || (delta < 1000) || !data_ready ) return 0; //still waiting answer 

    *data=tmp&0xFFFF;
    tmp>>=16;
    *id=tmp&0xFF;
    tmp>>=12;
    *type=tmp&0x7;

    return 1; //all ok

}
void OpenTherm::update(){
    update(255);
}

void OpenTherm::update(uint8_t next){
    uint8_t type=0,id=0;
    uint16_t data=0;
		
			
    if (state == 125) {
        communicate(0,state,0);
        state=0;
        return;
    }
    
		
    switch(complete(&type,&id,&data)){
       case 0: return; //still waiting;
        break;
       case 1:  // nothing to do - all ok
        break;
       default: // must be 2- anyway start communication again;
            type=0x2; //invalidating answer;
       break;
    }
        

  
    switch(id){
        case 0: 
            if(!(type&0x2)) status=data&0xFF;
            data=0;
            if(status & 0x1) state=5;
            else state=17;
            type=0;
        break;

        case 1: state=0;
        break;
        case 2: state=0;
        break;
        case 3:
            if(!(type&0x2)) {
            member_id=data&0xFF;
            sl_cfg=(data>>8)&0xFF;}
            type=1;
            state=0;
        break;
        case 5:
        	if(!(type&0x2)) fault=(data>>8)&0xFF;
            state=0;
        break;
        case 14:
            if(!(type&0x2))max_modulation=data>>8;
            type=0;
            state=0;
        break;
        case 15:
            if(!(type&0x2)){
            min_modulation=data&0xFF;
            max_capacity=(data>>8)&0xFF;}
            type=0;
            state=48;
        break;
        case 17:
            if(!(type&0x2)) modulation=data>>8;
            if(status&0x2) state=25;
            else state=26;
            data=0;
        break;
        case 18:
            if(!(type&0x2)) CH_water_pressure=OT_to_float(data);
            state=19;
            type=0;
        break;
        case 19:
            if(!(type&0x2)) DHW_flow=OT_to_float(data);
            state=28;
            type=0;
        break;
        case 25:
            if(!(type&0x2)) CH=OT_to_float(data);
            state=0;
        break;
        case 26:
            if(!(type&0x2)) DHW=OT_to_float(data);
            state=0;
        break;
        case 28:
            if(!(type&0x2)) CH_return_temp=OT_to_float(data);
            state=15;
            type=0;
        break;
        case 48:
            if(!(type&0x2)){
            DHW_min_lim=data&0xFF;
            DHW_max_lim=(data>>8)&0xFF;}
            type=0;
            state=49;
        break;
        case 49:
            if(!(type&0x2)){
            CH_min_lim=data&0xFF;
            CH_max_lim=(data>>8)&0xFF;}
            type=0;
            state=18;
        break;
        case 56:
            if(!(type&0x2)) target_DHW=OT_to_float(data);
            state=0;
        break;
        case 57:
            if(!(type&0x2)) CH_max=OT_to_float(data);
            state=0;
        break;
        case 116:
            if(!(type&0x2)) burner_starts=data;
            state=117;
            data=0;
            type=0;
        break;
        case 117:
            if(!(type&0x2)) CH_pump_starts=data;
            state=118;
            data=0;
            type=0;
        break;
        case 118:
            if(!(type&0x2)) DHW_pump_starts=data;
            state=119;
            data=0;
            type=0;
        break;
        case 119:
            if(!(type&0x2)) DHW_burner_starts=data;
            state=120;
            data=0;
            type=0;
        break;
        case 120:
            if(!(type&0x2)) burner_op_hours=data;
            state=121;
            data=0;
            type=0;
        break;
        case 121:
            if(!(type&0x2)) CH_pump_op_hours=data;
            state=122;
            data=0;
            type=0;
        break;
        case 122:
            if(!(type&0x2)) DHW_pump_op_hours=data;
            state=123;
            data=0;
            type=0;
        break;
        case 123:
            if(!(type&0x2)) DHW_burner_op_hours=data;
            state=116;
            data=0;
            type=0;
        break;
        case 125:
            if(!(type&0x2)) slave_ver=OT_to_float(data);
            state=3;
            data=0;
            type=0;
        break;
        
            
    }

    if (next==255 && state==0) next=0; //for all non-targeted updates, which executed at the chain end, go to default chain

    switch(next){
        case 0:
            type=0;
            state=0;
            data=(DHW_enabled<<1|CH_enabled)<<8;
        break;
        case 1: 
            data=float_to_OT(target_CH);
            type=1;
        break;
        case 14:
        		data=max_modulation<<8;
            type=1;
        break;
        case 18: 
            data=0;
            type=0;
        break;
        case 56: 
            data=float_to_OT(target_DHW);
            type=1;
        break;
        case 57:
            data=float_to_OT(CH_max); 
            type=1;
        break;
        case 116:
            data=0;
            type=0;
        break;

    }
    communicate(type,(next==255)?state:next,data); //for all non-targeted updates (next==255) go to next state. For targeted - execute target.
    
}
