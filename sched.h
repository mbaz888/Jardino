/*
 * 
 *  sched.cpp
 *
 *
 */

#ifndef _SCHED_H_
#define _SCHED_H_

#include <TimeLib.h>

#include <types.h>


#define EEPROM_SIGNATURE   67
#define EEPROM_TIME_AREA   32
#define EEPROM_CONFIG_AREA   128
#define EEPROM_USER_AREA   256


#define LEN_ZONE_NAME 16 

struct ZoneInfo{
	int  duration;
  int  curr_duration;
  int  countdown;
	time_t  time_start;
	time_t  time_end;
	int  relay;
	bool active;

};


struct ProgramInfo{
  int      id;
	int      start_time;
  uint8_t  hour_start;
  uint8_t  minute_start;
	ZoneInfo zones[MAX_ZONES];
	int      dow;
	int      repeat_days;
  int      correction;
	bool     active;
};


bool  sched_loop();
void  sched_setup(void);
void  sched_manual(int zone,int duration);
void  sched_manuals(int zone[]);
void  sched_cycle(int program);
void  sched_cancel();
void  sched_set_program(int n,bool active,int hh,int mm,int days,int zones[]);
void  sched_set_active(int n,bool active);
void  sched_set_correction(int correction);
void  sched_set_delay_zones(int delayZ);
const ProgramInfo* sched_get_program(int np);
const ProgramInfo* sched_get_program_ordered(int np);
const ZoneInfo* sched_get_current();
void sched_get_durations(int dur[]);
const char* sched_get_zone_name(int nz);
void sched_set_zone_name(const char* s,int nz);

time_t midnight(time_t);

uint8_t get_relay_active();
void    set_all_relays_off();
bool    watering_running();
#endif
