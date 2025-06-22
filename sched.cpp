
/*
 *  sched.cpp
 *
 *
 */
 /*



*/
#include <Arduino.h>   
#include <WiFi.h>
#include <EEPROM.h>
#include <TimeLib.h>
#include <Time.h>
#include <Timezone.h> //https://github.com/JChristensen/Timezone

// set to true to reprogram eeprom
bool restore_factory = false; 



//Central European Time (Frankfurt, Paris)
TimeChangeRule CEST_s = {"CEST", Last, Sun, Mar, 2, 120};     //Central European Summer Time
TimeChangeRule CET_s = {"CET ", Last, Sun, Oct, 3, 60};       //Central European Standard Time

Timezone CE_s(CEST_s, CET_s);


#include "types.h"



const int  MANUAL_PROGRAM = MAX_PROGRAMS;

ProgramInfo programs[MAX_PROGRAMS+1]; // add manual program
ProgramInfo* programs_order[MAX_PROGRAMS];

int  nprograms = 0;

int  nzones_next_sched = 0;
ZoneInfo zones_next_sched[MAX_ZONES];

typedef char ZoneName[LEN_ZONE_NAME] ;
ZoneName zone_names[MAX_ZONES];

uint8_t active_relay = 0;
    
int time_count = 0;
int relay_data = 0;
int zoffset = 0;

int delay_bz = 2;
int g_correction = 0;
int prg_running = -1;

// ======================================================
// ======================================================

//extern time_t utc_time;
// ======================================================

int  dow(time_t t)
{
  int dow = t + zoffset;
  dow = dow / 86400;
  dow = dow - 3;
  dow = dow % 7;
  return(1 << dow);  // day of week bitmask in proper timezone
}
// ======================================================

time_t  midnight(time_t t)
{
  time_t mid = t + zoffset;
  mid /= 86400;
  mid *= 86400; // midnight current day proper timezone
  mid -= zoffset;  // local midnight in UTC
  return mid;
}

// ======================================================
// TEST
time_t  time_fake(time_t* v)
{
    //return utc_time;
    return 0;
    *v = 0;
}



// ======================================================

time_t toNextUTCTime(uint8_t hh_local,uint8_t mm_local)
{
  time_t now;
  time(&now);
  time_t t =  CE_s.toUTC(midnight(now) + (hh_local * 60 * 60) + (mm_local * 60));

  if (t < now) t += (60*60*24);

  return t;
}


// ======================================================


void  set_relay(int num, int onoff, int duration)
{
    if (num < 1 || num > 8) return;

  active_relay = num;

}


bool  get_relay(int num)
{
  return (active_relay == num);
}


uint8_t  get_relay_active()
{
  return active_relay;
}

void  set_all_relays_off()
{

  active_relay = 0;
}

void init_relay()
{
  active_relay = 0;
}


bool watering_running()
{
  return (active_relay > 0);
}

// ============================================
void  save_config()
{
    EEPROM.begin(512); // EEPROM emulated in flash, 512 bytes
    int addr = EEPROM_USER_AREA;
    EEPROM.write(addr,EEPROM_SIGNATURE); // Signature
    addr++;
    
    for (int p = 0; p < MAX_PROGRAMS; p++)
    {
      EEPROM.write(addr++,(programs[p].active?1:0));  
      EEPROM.write(addr++,programs[p].repeat_days); 
      EEPROM.write(addr++,programs[p].hour_start);
      EEPROM.write(addr++,programs[p].minute_start);  
      for (int i = 0; i < MAX_ZONES; i++)
      {
        EEPROM.write(addr++,programs[p].zones[i].duration);
      }
    }
    
    for (int nz = 0; nz < MAX_ZONES; nz++)
    {
      for (int c = 0; c < LEN_ZONE_NAME; c++)
        EEPROM.write(addr++,zone_names[nz][c]);
    }
    
    EEPROM.end();
}

// ============================================

void  load_config()
{
    int i,p;
  bool config_saved = false;

  EEPROM.begin(512); // EEPROM emulated in flash, 512 bytes
  int addr = EEPROM_USER_AREA;
  byte v;
  v = EEPROM.read(addr);
  if ( v == EEPROM_SIGNATURE && !restore_factory)
  {
    config_saved = true;
    addr++;
    for (int p = 0; p < MAX_PROGRAMS; p++)
    {    
      programs[p].dow = 0xFF;
      programs[p].id  = p+1;
      programs[p].active = (EEPROM.read(addr++) > 0);
      programs[p].repeat_days  = EEPROM.read(addr++);
      programs[p].hour_start   = EEPROM.read(addr++);
      programs[p].minute_start = EEPROM.read(addr++);
      programs[p].start_time   = toNextUTCTime (programs[p].hour_start,programs[p].minute_start);
 
      Serial.printf("EEPROM read, program %d start at %02d:%02d  Active: %d\r\n",
            programs[p].id,programs[p].hour_start,programs[p].minute_start,programs[p].active);
      for (int i = 0; i < MAX_ZONES; i++)
      {        
        programs[p].zones[i].active = true;
        programs[p].zones[i].relay = i+1;
        programs[p].zones[i].duration = EEPROM.read(addr++);
        if (programs[p].zones[i].duration == 0) 
          programs[p].zones[i].active = false;
        programs[p].zones[i].countdown = 0;
      }
    }

    for (int nz = 0; nz < MAX_ZONES; nz++)
    {
      for (int c = 0; c < LEN_ZONE_NAME; c++)
        zone_names[nz][c] = EEPROM.read(addr++);
    }
  }
  EEPROM.end();

  if (!config_saved)
  {
    // Default values
    for (p = 0; p < MAX_PROGRAMS; p++)
    {
      programs[p].dow = 0xFF;
      programs[p].id  = p+1;
      programs[p].active = false;
      programs[p].repeat_days = 1;
      for (i = 0; i < MAX_ZONES; i++)
      {
        programs[p].zones[i].relay = i+1;
      }
    }

    Serial.println("Default values");
    g_correction = 0;
    programs[0].active = true;
    programs[0].hour_start = 9;
    programs[0].minute_start = 30;
    programs[0].start_time = toNextUTCTime (programs[0].hour_start,programs[0].minute_start);
    programs[0].start_time /= 60;
    programs[0].start_time *= 60;
    programs[0].correction = 0;
  
    programs[0].zones[0].duration = 2 * 60;
    programs[0].zones[0].active = true;
    
    programs[0].zones[1].duration = 2 * 60;
    programs[0].zones[1].active = true;

    programs[0].zones[2].duration = 2 * 60;
    programs[0].zones[2].active = true;

    programs[0].zones[3].duration = 2 * 60;
    programs[0].zones[3].active = true;
   
    programs[0].zones[4].duration = 2 * 60;
    programs[0].zones[4].active = true;

    programs[0].zones[5].duration = 2 * 60;
    programs[0].zones[5].active = true;

    for (int nz = 0; nz < MAX_ZONES; nz++)
      sprintf(zone_names[nz],"JARDIN %d",nz+1);
      
    save_config();
  }

  for (int p = 0; p < MAX_PROGRAMS; p++)
    programs_order[p] = &programs[p];
}

// =================================================

// =================================================
void compute_times();


// =================================================

void  sched_set_active(int n,bool active)
{
  if (n < 0 || n >= MAX_PROGRAMS) return;
  
  programs[n].active = active;

  compute_times();
}

// =================================================

void  do_reset()
{
  set_all_relays_off();
}


// =============================================================
// =============================================================


time_t next_sched_start;
time_t next_sched_end;

// =============================================================

void compute_times()
{

    // Initialite next sched structure.
    for (int nz = 0; nz < MAX_ZONES; nz++)
    {
      zones_next_sched[nz].time_start = 0x7FFFFFFF;
      zones_next_sched[nz].active = false;
    }
    next_sched_start = 0x7FFFFFFF;
    next_sched_end = 0;
  
    time_t now;
    time(&now);
 
    for (int p = 0; p < MAX_PROGRAMS+1; p++)
    {
      if (!programs[p].active) continue;

      time_t timep = programs[p].start_time;

      if (p != MANUAL_PROGRAM) // Only automatic programs are updated
      {
          int failsafe = 0;
          // while (timep < now && (dow(timep) & programs[p].dow == 0) && failsafe++ < 100)
          while (timep < now && failsafe++ < 100)
          {
              timep += (86400 * programs[p].repeat_days);
          }
    
          programs[p].start_time = timep;
      }
    
      time_t end_time = 0;
      for (int nz = 0; nz< MAX_ZONES ; nz++)
      {
        if (programs[p].zones[nz].active)
        {
           // Adjust duration with weather correction
           programs[p].zones[nz].curr_duration = programs[p].zones[nz].duration + 
                        ((programs[p].zones[nz].duration * g_correction)/100);
                        //((programs[p].zones[nz].duration * programs[p].correction)/100);
                          
           programs[p].zones[nz].time_start = timep;
           timep += programs[p].zones[nz].curr_duration;      
           programs[p].zones[nz].time_end   = timep;
           timep += delay_bz;

           if (programs[p].zones[nz].time_end > end_time)
             end_time = programs[p].zones[nz].time_end;
         }
      }

      // ===========================================
      // Computes  next schedule
      // ===========================================
      if (next_sched_start > programs[p].start_time)
      {
        // Copy zones into next sched;
        for (int nz = 0; nz < MAX_ZONES ; nz++)
        {
          memcpy((void*)&zones_next_sched[nz],(void*)&programs[p].zones[nz],sizeof(ZoneInfo));
        }    
        next_sched_start = programs[p].start_time;
        next_sched_end   = end_time;
        //Serial.printf("Next start program %d  %d -> %d\n",p,next_sched_start,next_sched_end);

      }    
    }

    // ===========================================
    // Computes  programs order
    // ===========================================

    int na = 0;

    // Initialize the order-array with the unordered active programs pointers
    for (int p = 0; p < MAX_PROGRAMS; p++)
    {
      if (programs[p].active)
        programs_order[na++] = &programs[p];
    }


    // Order active programs working on order-array
    for (int x = 0; x < na; x++)
    {   
      for (int y = x+1; y < na; y++)  
      {
        if (programs_order[x]->start_time > programs_order[y]->start_time)
        {
          ProgramInfo* ant  = programs_order[x];
          programs_order[x] = programs_order[y];
          programs_order[y] = ant;       
        }
      }
    }

    // Fill the remaining positions with the no active programs
    for (int p = 0; p < MAX_PROGRAMS ; p++)
    {      
      if (!programs[p].active && na < MAX_PROGRAMS)
        programs_order[na++] = &programs[p];
    }    

}

// =============================================================

void sched_set_program(int np,bool active,int hh,int mm,int days,int zones[])
{
   int p = np - 1;
 
   if (p < 0 || p >= MAX_PROGRAMS) return;
  
   programs[p].dow = 0xFF;
   programs[p].active = active;
   programs[p].repeat_days  = days;
   programs[p].hour_start   = hh;
   programs[p].minute_start = mm;
   programs[p].start_time = toNextUTCTime (hh,mm);

   for (int z = 0; z < MAX_ZONES; z++)
   {
     programs[p].zones[z].duration = min(MAX_DURATION,zones[z]);
     programs[p].zones[z].active   = (zones[z] > 0 ? true : false);
   }
  
   compute_times();
  
   save_config();

   Serial.printf("New start time %d \n",programs[p].start_time);
}

// =============================================================


void sched_manual(int z,int duration)
{
    if ( z <= 0 or z > MAX_ZONES) return;
    if ( duration > MAX_DURATION) return;

    int zone = z - 1;
    
    // All zones OFF  
    for (int nz = 0; nz< MAX_ZONES ; nz++)
    {
      zones_next_sched[nz].active = false;   
    }

    time_t now;
    time(&now);
    zones_next_sched[zone].active = true;
    zones_next_sched[zone].relay  = zone+1;
    zones_next_sched[zone].duration  = duration;
    zones_next_sched[zone].curr_duration  = duration; // Not adjust
    zones_next_sched[zone].time_start = now + 1;
    zones_next_sched[zone].time_end = zones_next_sched[zone].time_start  + duration;
    next_sched_start = zones_next_sched[zone].time_start;
    next_sched_end   = zones_next_sched[zone].time_end;
}

// =============================================================

void  sched_manuals(int zones[])
{

  time_t now;
  time(&now);
  programs[MANUAL_PROGRAM].dow = 0xFF;
  programs[MANUAL_PROGRAM].active = true;
  programs[MANUAL_PROGRAM].repeat_days = 1;
  programs[MANUAL_PROGRAM].start_time = now + 1;

  
  for (int z = 0; z < MAX_ZONES; z++)
  {
    programs[MANUAL_PROGRAM].zones[z].relay    = z+1;
    programs[MANUAL_PROGRAM].zones[z].duration = min(MAX_DURATION,zones[z]);
    programs[MANUAL_PROGRAM].zones[z].active   = (zones[z] > 0 ? true : false);
  }
  
  compute_times();
}

// =============================================================

void  sched_cycle(int prg)
{
  
  time_t now;
  time(&now);
  
  if (prg <= 0 or prg > MAX_PROGRAMS) return;
  int p = prg - 1;
  
  if (!programs[p].active) return;

  // Force that this program schedule will be the next schedule;

  time_t ptime = now + 2; // Add 2 seconds to allow main loop to react 
  time_t end_time = ptime;
  
  next_sched_start = ptime;
  
  for (int nz = 0; nz < MAX_ZONES ; nz++)
  {
      memcpy((void*)&zones_next_sched[nz],(void*)&programs[p].zones[nz],sizeof(ZoneInfo));

      if (zones_next_sched[nz].active)
      {
        zones_next_sched[nz].curr_duration = zones_next_sched[nz].duration + 
                          ((zones_next_sched[nz].duration * programs[p].correction)/100);

        zones_next_sched[nz].time_start = ptime;
        ptime += zones_next_sched[nz].curr_duration;
        zones_next_sched[nz].time_end   = ptime;
        end_time = ptime;
        ptime += delay_bz;

      }  
  }  

  next_sched_end   = end_time;
  
}

// =============================================================

void  sched_cancel()
{
  // Cancel current scheduler ( including manual) 
  programs[MANUAL_PROGRAM].active = false;
  
  compute_times();

  set_all_relays_off();
}

// =============================================================

void  sched_set_correction(int correction)
{
  for (int p = 0; p < MAX_PROGRAMS; p++)
    programs[p].correction = correction;

  if (correction != g_correction) {
    g_correction = correction;
    compute_times();
  }
  
 
}

// =============================================================

const ProgramInfo* sched_get_program(int np)
{

  int p = np - 1;
 
  if (p < 0 || p >= MAX_PROGRAMS) return NULL;

  return &(programs[p]);
  
  /*
    for (int p = 0; p < MAX_PROGRAMS; p++)
      if (programs[p].id == np) return &(programs[p]);

    return NULL;
*/
}
// =============================================================
const ProgramInfo* sched_get_program_ordered(int order) // 0 -> first
{
  if (order < 0 or order >= MAX_PROGRAMS) return NULL;

  return programs_order[order];
}

// =============================================================

void  sched_set_delay_zones(int delayZ)
{
  if (delayZ < 0 or delayZ > 10) return;
  delay_bz = delayZ;    
  compute_times();
}


// =============================================================

const ZoneInfo* sched_get_current()
{
  if (prg_running == -1) return NULL;
  return &zones_next_sched[prg_running];
}

// =============================================================

void sched_get_durations(int dur[])
{
  for (int nz = 0; nz< MAX_ZONES ; nz++)
  {
    ZoneInfo* pzone = &zones_next_sched[nz];
    
    if (!pzone->active)
      dur[nz] = 0;
    else
      dur[nz] = pzone->curr_duration; // 0 if raining ( -100%)
  }
}


// =============================================================

const char* sched_get_zone_name(int nz)
{
  if (nz <= 0 or nz > MAX_ZONES) return "";
  return zone_names[nz-1];
}

// =============================================================

void sched_set_zone_name(const char* s,int nz)
{
    if ( nz <= 0 or nz > MAX_ZONES) return;
    strncpy(zone_names[nz-1],s,LEN_ZONE_NAME);
}

// =============================================================
// Must be called each second.

bool  sched_loop()
{
  time_t now;
  time(&now);
  prg_running = -1;
 

  //Serial.printf("NOW %d\n",now);
  for (int nz = 0; nz< MAX_ZONES ; nz++)
  {
    ZoneInfo* pzone = &zones_next_sched[nz];
    
    if (!pzone->active) continue;

   /*
    Serial.printf("---------- \r\nNZ %d  [%d] %d->%d\r\n",
           nz+1,
           now % (3600*24),
           pzone->time_start % (3600*24),
           pzone->time_end % (3600*24));
           */

    if (pzone->time_start <= now  && now < pzone->time_end) 
    {
        prg_running = nz;
        
        active_relay = pzone->relay;
        pzone->countdown = pzone->time_end - now;
        //Serial.printf("Relay %d ON \r\n",pzone->relay);
        break; // Check !! with this we prevent overlapped zone schedules 
    }
    else
    {
        pzone->countdown = 0;
        if (pzone->relay == active_relay)
          active_relay = 0;
        //Serial.printf("Relay %d OFF \r\n",pzone->relay);
    } 
  }

  if (now > next_sched_end) // Finish current program
  {
    programs[MANUAL_PROGRAM].active = false; // Always disable Manual program
    compute_times();
  }
  
  // return true if we are in the schedule period
  return (now >= next_sched_start && now < next_sched_end);

}
// =============================================================


void  sched_setup(void)
{
    init_relay();
    load_config();
    compute_times();
}
