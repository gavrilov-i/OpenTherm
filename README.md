# OpenTherm
Opentherm library for Arduino
Sucessfully tested on Ferolli Domiproject F24.
Setting CH/DHW Setpont. Resetting faults, getting status/modulation level/temperatures, etc.
http://habrahabr.ru/post/251539


For sending simple request just call communicate(MSG-TYPE,DATA-ID,DATA); and after 1s take answer complete(&type,&id,&data):

uint8_t type,id;
uint16_t data;

communicate(1,56,10240); //set DHW setpoint to 40.0 deg

delay(1000);

complete(&type,&id,&data); // if accepted, type=5,id=56, data=10240;
