// BMS MASTER
//
// @author Guillaume Soulard
// @date 26-7-18
//
// Definition of extern declarations
//

#include "defines.h"

uint16_t g_temperatureSensorIdx = 0;

struct EventCount g_EventCount;
volatile uint16_t g_slaveMeasurementCountMs;
volatile uint16_t g_saveDataEepromSeconds;
uint16_t g_pecFailCount;
volatile uint16_t g_errorTimerMs;
struct MasterBMS g_masterBMS;
int16_t g_current_probe_offset;
bool g_close_interlock_after_next_valid_error_check;
struct Charger g_Charger;
uint64_t g_error_status;
