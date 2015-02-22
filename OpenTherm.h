#define MSG_LENGTH 32 //32 bit 
#define TICKS_PER_MS F_CPU/64/1000 

#include <Arduino.h>
#include <inttypes.h>
#include <avr/interrupt.h>


#define OT_STATUS_FAULT 0x1
#define OT_STATUS_CH 0x2
#define OT_STATUS_DHW 0x4

#define OT_FAULT_SERVICE 0x1
#define OT_FAULT_LOW_WATER 0x4
#define OT_FAULT_GAS 0x8
#define OT_FAULT_AIR_PRESSURE 0x10
#define OT_FAULT_WATER_OV_TEMP 0x20


struct ot_init_settings{
    bool CH_enabled;
    bool DHW_enabled;
    uint8_t CH_max_temp;
    float CH_temp;
    uint8_t DHW_temp;
    uint8_t max_modulation;
    uint8_t reserved[10];
};


class OpenTherm {
  private:
    void send();
    void abort();
    void inline receive();
    float OT_to_float(uint16_t);
    uint16_t float_to_OT(float);
    uint32_t ts;
    uint8_t state;
    volatile uint8_t* tx_port;
    uint8_t tx_bitmask;
    volatile uint8_t* rx_port;
    uint8_t rx_bitmask;
    volatile uint8_t* rx_pcmsk;
    uint8_t rx_pcie;
    volatile uint32_t buf; // Opentherm message
		volatile uint8_t rx; //receive flag 1 - receiving 0 - transmitting
		volatile uint8_t first; //first bit
		volatile uint8_t length; //data length in bits
		volatile uint8_t data_ready; //data ready flag
		volatile uint8_t parity;



  public:

    uint8_t status;
    bool CH_enabled;
    bool DHW_enabled;
    uint8_t member_id;
    uint8_t sl_cfg;
    float DHW;
    float CH;
    float target_DHW;
    float target_CH;
    uint8_t modulation;
    uint8_t fault;
    float slave_ver;
    float CH_water_pressure;
    float DHW_flow;
    float CH_return_temp;
    float CH_max;
    uint8_t max_modulation;
    uint8_t max_capacity;
    uint8_t min_modulation;
    uint8_t DHW_max_lim;
    uint8_t DHW_min_lim;
    uint8_t CH_max_lim;
    uint8_t CH_min_lim;

    uint16_t burner_starts;
    uint16_t CH_pump_starts;
    uint16_t DHW_pump_starts;
    uint16_t DHW_burner_starts;
    uint16_t burner_op_hours;
    uint16_t CH_pump_op_hours;
    uint16_t DHW_pump_op_hours;
    uint16_t DHW_burner_op_hours;    



    OpenTherm(uint8_t,uint8_t);
    void update();
    void update(uint8_t);
    void begin(ot_init_settings*);
    void communicate(const uint8_t ,const uint8_t ,const uint16_t ); //send packet with specified type,id and data. answer will be in same variables when complete()=1
    uint8_t complete(uint8_t *,uint8_t *,uint16_t *); //0- answer not received yet, 1-answer received
    bool extIntHandler();
    void timer2CompAHandler();
    void timer2CompBHandler();
};
