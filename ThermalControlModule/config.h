#define DELAY_STATUSMSG 1000    // Heartbeat message every 1 second

#define DRIVEUNIT_MAX_F 120
#define DRIVEUNIT_HIMAX_F 180
#define BATTERY_MIN_F 50
#define BATTERY_MAX_F 90
#define BATTERY_HIMAX_F 110
#define CHARGER_MAX_F 100
#define CHARGER_HIMAX_F 150
#define DCDC_MAX_F 100
#define DCDC_HIMAX_F 150
#define LOOP_TEMP_DIFF 10    // Difference in inner and outer loop temps to activate joining

// CAN communications and interrupt pin
// (definition included here as wiring guide only)
#define PIN_COMMS_SCK 50
#define PIM_COMMS_MISO 51
#define PIM_COMMS_MOSI 52
#define PIN_IN_CAN_INTERRUPT 2

// 5v PWM Control line for the high voltage heater
#define PIN_OUT_HEATER_PWM 4

// 5v PWM Control line for the high voltage AC compressor
#define PIN_OUT_ACCOMP_PWM 5
#define PIN_OUT_CHILLER_DIG 22

// Pairs of lines for the collant pumps
// 5v 2Hz digital PWM signal line
// 5v analog feedback line indicating position
#define PIN_IN_PUMP1_ANA 0
#define PIN_OUT_PUMP1_DIG 6
#define PIN_IN_PUMP2_ANA 1
#define PIN_OUT_PUMP2_DIG 7
#define PIN_IN_PUMP3_ANA 2
#define PIN_OUT_PUMP3_DIG 8

// Fan Control
#define PIN_OUT_FAN_1 29
#define PIN_OUT_FAN_2 30
#define FAN_DELAY_MS 3000     // Time delay in ms between the start of the first fan and the start of the second (to reduce the sudden AMP load)

// Coolant Valves, 5v signal lines to 12v H Bridge
#define PIN_OUT_PWM_BYPASS_1 9
#define PIN_OUT_OPEN_BYPASS_1 23
#define PIN_OUT_CLOSE_BYPASS_1 24
#define PIN_OUT_PWM_BYPASS_2 9
#define PIN_OUT_OPEN_BYPASS_2 25
#define PIN_OUT_CLOSE_BYPASS_2 26
#define PIN_OUT_PWM_LOOP 9
#define PIN_OUT_OPEN_LOOP 27
#define PIN_OUT_CLOSE_LOOP 28

// Minimum Run Times (in ms)
#define FAN_MIN_RUN_MS 10000
#define HEATER_MIN_RUN_MS 30000
#define ACCOMP_MIN_RUN_MS 30000
#define RADIATOR_MIN_RUN_MS 30000
#define LOOP_MIN_RUN_MS 30000
