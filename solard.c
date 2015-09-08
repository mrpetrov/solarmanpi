/*
    * solard.c
    *
    * Raspberry Pi solar manager which uses 1wire and GPIO.
    * Plamen Petrov <plamen.sisi@gmail.com>
    *
    * solard is Plamens's custom solar controller, based on the Raspberry Pi 2.
    * Data is gathered and logged every 10 seconds from 4 DS18B20 waterproof sensors,
    * controled are 4 relays via GPIO, and a GPIO pin is read for managing the power
    * to Grundfoss UPS2 pumps, which if left alone block on power switch to battery.
    * Remedy is to leave the pumps off for a couple of seconds on power failure (the
    * UPS switches to battery fine), and then turn the pump back on. Log data is in
    * CSV format, to be picked up by some sort of data collection/graphing tool, like
    * collectd or similar.
    * The daemon is controlled via its configuration file, which if need be - can
    * be requested to be re-read while running. This is done by sending SIGUSR1
    * signal to the daemon process. The event is noted in the log file.
    * The logfile itself can be "grep"-ed for "ALARM:" to catch and notify of
    * notable events, recorded by the daemon.
*/

/* example for /etc/solard.cfg config file (these are the DEFAULTS):
    # soldard.cfg
    # version 1.0
    # $date-id

    # mode: 0=ALL OFF; 1=AUTO; 2=AUTO+HEAT HOUSE BY SOLAR; 3=MANUAL PUMP1 ONLY;
    # mode: 4=MANUAL PUMP2 ONLY; 5=MANUAL HEATER ONLY; 6=MANAUL PUMP1+HEATER
    # mode: 7=AUTO ELECTICAL HEATER ONLY - this one obeys start/stop hours
    # mode: 8=AUTO ELECTICAL HEATER ONLY, DOES NOT CARE ABOUT SCHEDULE !!!
    mode=1

    # wanted_T: the desired temperature of water in tank
    wanted_T=40

    # these define allowed hours to use electric heat
    use_electric_start_hour=3
    use_electric_stop_hour=4

    # this tells to default pump1 to ON in idles
    keep_pump1_on=0

    # control the use of solar pump
    use_pump2=1

    # day of month to reset power used counters
    day_to_reset_Pcounters=7
*/

#define SOLARDVERSION    "3.5-rc1 2015-09-08"

#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <signal.h>
#include <ctype.h>
#include <time.h>

#define RUNNING_DIR     "/tmp"
#define LOCK_FILE       "/run/solard.pid"
#define LOG_FILE        "/var/log/solard.log"
#define DATA_FILE       "/run/shm/solard_data.log"
#define TABLE_FILE      "/run/shm/solard_current"
#define CONFIG_FILE     "/etc/solard.cfg"
#define POWER_FILE      "/var/log/solard_power"

#define BUFFER_MAX 3
#define DIRECTION_MAX 35
#define VALUE_MAX 50
#define MAXLEN 80

#define IN  0
#define OUT 1

#define LOW  0
#define HIGH 1

/* Define GPIO pins used for control */
#define PUMP1  17 /* P1-11 */
#define PUMP2  18 /* P1-12 */
#define VALVE  27 /* P1-13 */
/* P1-14 is GROUND */
#define HEAT   22 /* P1-15 */

#define POWER  25 /* P1-22 */

/* Maximum difference allowed for data received from sensors between reads, C */
#define MAX_TEMP_DIFF        5

/* Number of all sensors to be used by the system */
#define TOTALSENSORS         4

/* Define the DS18B20 sensors paths to read temps from */
#define SENSOR1 "/sys/bus/w1/devices/28-041464764cff/w1_slave"
#define SENSOR2 "/sys/bus/w1/devices/28-041464403bff/w1_slave"
#define SENSOR3 "/sys/bus/w1/devices/28-04146448f3ff/w1_slave"
#define SENSOR4 "/sys/bus/w1/devices/28-0214608d40ff/w1_slave"

char *sensor_paths[] = { SENSOR1, SENSOR2, SENSOR3, SENSOR4 };

/* var to keep track of read errors, so if a threshold is reached - the
program can safely shut down everything, send notification and bail out;
initialised with borderline value to trigger immediately on errors during
start-up; the program logic tolerates 1 minute of missing sensor data */
unsigned short sensor_read_errors[TOTALSENSORS] = { 4, 4, 4, 4 };

/* current sensors temperatures - e.g. values from last read */
float sensors[9] = { -200, 10, 12, 20, 10, 10, 12, 20, 10 };

/* and sensor name mappings */
#define   Tkotel                sensors[1]
#define   Tkolektor             sensors[2]
#define   TboilerHigh           sensors[3]
#define   TboilerLow            sensors[4]
#define   TkotelPrev            sensors[5]
#define   TkolektorPrev         sensors[6]
#define   TboilerHighPrev       sensors[7]
#define   TboilerLowPrev        sensors[8]

/* current controls state - e.g. set on last decision making */
short controls[7] = { -1, 0, 0, 0, 0, 0, 0 };

/* and control name mappings */
#define   CPump1                controls[1]
#define   CPump2                controls[2]
#define   CValve                controls[3]
#define   CHeater               controls[4]
#define   CPowerByBattery       controls[5]
#define   CPowerByBatteryPrev   controls[6]

/* controls state cycles - zeroed on change to state */
long ctrlstatecycles[5] = { -1, 150000, 150000, 2200, 2200 };

#define   SCPump1               ctrlstatecycles[1]
#define   SCPump2               ctrlstatecycles[2]
#define   SCValve               ctrlstatecycles[3]
#define   SCHeater              ctrlstatecycles[4]

float TotalPowerUsed;
float NightlyPowerUsed;

/* solard keeps track of total and night tariff watt-hours electrical power used */
/* night tariff is between 23:00 and 06:00 */
/* constants of Watts of electricity used per 10 secs */
#define   HEATERPPC         8.340
#define   PUMPPPC           0.140
#define   VALVEPPC          0.006
#define   SELFPPC           0.022
/* my boiler uses 3kW per hour, so this is 0,00834 kWh per 10 seconds */
/* this in Wh per 10 seconds is 8.34 W */

/* NightEnergy (NE) start and end hours variables - get recalculated every days */
unsigned short NEstart = 20;
unsigned short NEstop  = 11;

/* Nubmer of cycles (circa 10 seconds each) that the program has run */
unsigned long ProgramRunCycles  = 0;

/* timers - current hour and month vars - used in keeping things up to date */
unsigned short current_timer_hour = 0;
unsigned short current_month = 0;

struct structsolard_cfg
{
    char    mode_str[MAXLEN];
    int     mode;
    char    wanted_T_str[MAXLEN];
    int     wanted_T;
    char    use_electric_start_hour_str[MAXLEN];
    int     use_electric_start_hour;
    char    use_electric_stop_hour_str[MAXLEN];
    int     use_electric_stop_hour;
    char    keep_pump1_on_str[MAXLEN];
    int     keep_pump1_on;
    char    use_pump2_str[MAXLEN];
    int     use_pump2;
    char    day_to_reset_Pcounters_str[MAXLEN];
    int     day_to_reset_Pcounters;
}
structsolard_cfg;

struct structsolard_cfg solard_cfg;

short need_to_read_cfg = 0;

short just_started = 0;

/* FORWARD DECLARATIONS so functions can be used in preceding ones */
short
DisableGPIOpins();
/* end of forward-declared functions */

void
SetDefaultCfg() {
    strcpy( solard_cfg.mode_str, "1");
    solard_cfg.mode = 1;
    strcpy( solard_cfg.wanted_T_str, "40");
    solard_cfg.wanted_T = 40;
    strcpy( solard_cfg.use_electric_start_hour_str, "3");
    solard_cfg.use_electric_start_hour = 3;
    strcpy( solard_cfg.use_electric_stop_hour_str, "4");
    solard_cfg.use_electric_stop_hour = 4;
    strcpy( solard_cfg.keep_pump1_on_str, "0");
    solard_cfg.keep_pump1_on = 0;
    strcpy( solard_cfg.use_pump2_str, "1");
    solard_cfg.use_pump2 = 1;
    strcpy( solard_cfg.day_to_reset_Pcounters_str, "7");
    solard_cfg.day_to_reset_Pcounters = 7;
}

short
log_message(char *filename, char *message) {
    FILE *logfile;
    char file_string[300];
    char timestamp[30];
    time_t t;
    struct tm *t_struct;

    t = time(NULL);
    t_struct = localtime( &t );
    strftime( timestamp, sizeof timestamp, "%F %T", t_struct );
    sprintf( file_string, "%s%s", timestamp, message );
    logfile = fopen( filename, "a" );
    if ( !logfile ) return -1;
    fprintf( logfile, "%s\n", file_string );
    fclose( logfile );
    return 0;
}

/* this version of the logging function destroys the opened file contents */
void
log_msg_ovr(char *filename, char *message) {
    FILE *logfile;
    char file_string[300];
    char timestamp[30];
    time_t t;
    struct tm *t_struct;

    t = time(NULL);
    t_struct = localtime( &t );
    strftime( timestamp, sizeof timestamp, "%F %T", t_struct );
    sprintf( file_string, "%s%s", timestamp, message );
    logfile = fopen( filename, "w" );
    if ( !logfile ) return;
    fprintf( logfile, "%s\n", file_string );
    fclose( logfile );
}

/* trim: get rid of trailing and leading whitespace...
    ...including the annoying "\n" from fgets()
*/
char *
trim (char * s)
{
    /* Initialize start, end pointers */
    char *s1 = s, *s2 = &s[strlen (s) - 1];

    /* Trim and delimit right side */
    while ( (isspace (*s2)) && (s2 >= s1) )
    s2--;
    *(s2+1) = '\0';

    /* Trim left side */
    while ( (isspace (*s1)) && (s1 < s2) )
    s1++;

    /* Copy finished string */
    strcpy (s, s1);
    return s;
}

void
parse_config()
{
    int i = 0;
    char *s, buff[150];
    FILE *fp = fopen(CONFIG_FILE, "r");
    if (fp == NULL) {
        log_message(LOG_FILE," WARNING: Failed to open "CONFIG_FILE" file for reading!");
        } else {
        /* Read next line */
        while ((s = fgets (buff, sizeof buff, fp)) != NULL)
        {
            /* Skip blank lines and comments */
            if (buff[0] == '\n' || buff[0] == '#')
            continue;

            /* Parse name/value pair from line */
            char name[MAXLEN], value[MAXLEN];
            s = strtok (buff, "=");
            if (s==NULL) continue;
            else strncpy (name, s, MAXLEN);
            s = strtok (NULL, "=");
            if (s==NULL) continue;
            else strncpy (value, s, MAXLEN);
            trim (value);

            /* Copy into correct entry in parameters struct */
            if (strcmp(name, "mode")==0)
            strncpy (solard_cfg.mode_str, value, MAXLEN);
            else if (strcmp(name, "wanted_T")==0)
            strncpy (solard_cfg.wanted_T_str, value, MAXLEN);
            else if (strcmp(name, "use_electric_start_hour")==0)
            strncpy (solard_cfg.use_electric_start_hour_str, value, MAXLEN);
            else if (strcmp(name, "use_electric_stop_hour")==0)
            strncpy (solard_cfg.use_electric_stop_hour_str, value, MAXLEN);
            else if (strcmp(name, "keep_pump1_on")==0)
            strncpy (solard_cfg.keep_pump1_on_str, value, MAXLEN);
            else if (strcmp(name, "use_pump2")==0)
            strncpy (solard_cfg.use_pump2_str, value, MAXLEN);
            else if (strcmp(name, "day_to_reset_Pcounters")==0)
            strncpy (solard_cfg.day_to_reset_Pcounters_str, value, MAXLEN);
        }
        /* Close file */
        fclose (fp);
    }

    /* Convert strings to int */
    strcpy( buff, solard_cfg.mode_str );
    i = atoi( buff );
    solard_cfg.mode = i;
    strcpy( buff, solard_cfg.wanted_T_str );
    i = atoi( buff );
    if ( i ) solard_cfg.wanted_T = i;
    strcpy( buff, solard_cfg.use_electric_start_hour_str );
    i = atoi( buff );
    solard_cfg.use_electric_start_hour = i;
    strcpy( buff, solard_cfg.use_electric_stop_hour_str );
    i = atoi( buff );
    solard_cfg.use_electric_stop_hour = i;
    strcpy( buff, solard_cfg.keep_pump1_on_str );
    i = atoi( buff );
    solard_cfg.keep_pump1_on = i;
    strcpy( buff, solard_cfg.use_pump2_str );
    i = atoi( buff );
    solard_cfg.use_pump2 = i;
    strcpy( buff, solard_cfg.day_to_reset_Pcounters_str );
    i = atoi( buff );
    if ( i ) solard_cfg.day_to_reset_Pcounters = i;

    /* Prepare log message and write it to log file */
    if (fp == NULL) {
        sprintf( buff, " INFO: Using values: M=%d, Twanted=%d, ELH start=%d, stop=%d, keepP1on=%d, useP2=%d, resetPday=%d",\
        solard_cfg.mode, solard_cfg.wanted_T, solard_cfg.use_electric_start_hour, \
        solard_cfg.use_electric_stop_hour, solard_cfg.keep_pump1_on, solard_cfg.use_pump2, solard_cfg.day_to_reset_Pcounters );
        } else {
        sprintf( buff, " INFO: Read CFG file: M=%d, Twanted=%d, ELH start=%d, stop=%d, keepP1on=%d, useP2=%d, resetPday=%d",\
        solard_cfg.mode, solard_cfg.wanted_T, solard_cfg.use_electric_start_hour, \
        solard_cfg.use_electric_stop_hour, solard_cfg.keep_pump1_on, solard_cfg.use_pump2, solard_cfg.day_to_reset_Pcounters );
    }
    log_message(LOG_FILE, buff);
}

void
WritePersistentPower() {
    FILE *logfile;
    char timestamp[30];
    time_t t;
    struct tm *t_struct;

    t = time(NULL);
    t_struct = localtime( &t );
    strftime( timestamp, sizeof timestamp, "%F %T", t_struct );

    logfile = fopen( POWER_FILE, "w" );
    if ( !logfile ) return;
    fprintf( logfile, "# solard power persistence file written %s\n", timestamp );
    fprintf( logfile, "total=%3.3f\n", TotalPowerUsed );
    fprintf( logfile, "nightly=%3.3f\n", NightlyPowerUsed );
    fclose( logfile );
}

void
ReadPersistentPower() {
    float f = 0;
    char *s, buff[150];
    char totalP_str[MAXLEN];
    char nightlyP_str[MAXLEN];
    short should_write=0;
    strcpy( totalP_str, "0" );
    strcpy( nightlyP_str, "0" );
    FILE *fp = fopen(POWER_FILE, "r");
    if (fp == NULL) {
        log_message(LOG_FILE," WARNING: Failed to open "POWER_FILE" file for reading!");
        should_write = 1;
        } else {
        /* Read next line */
        while ((s = fgets (buff, sizeof buff, fp)) != NULL)
        {
            /* Skip blank lines and comments */
            if (buff[0] == '\n' || buff[0] == '#')
            continue;

            /* Parse name/value pair from line */
            char name[MAXLEN], value[MAXLEN];
            s = strtok (buff, "=");
            if (s==NULL) continue;
            else strncpy (name, s, MAXLEN);
            s = strtok (NULL, "=");
            if (s==NULL) continue;
            else strncpy (value, s, MAXLEN);
            trim (value);

            /* Copy data in corresponding strings */
            if (strcmp(name, "total")==0)
            strncpy (totalP_str, value, MAXLEN);
            else if (strcmp(name, "nightly")==0)
            strncpy (nightlyP_str, value, MAXLEN);
        }
        /* Close file */
        fclose (fp);
    }

    if (should_write) {
        log_message(LOG_FILE, " Creating missing power persistence data file...");
        WritePersistentPower();
    }
    else {
        /* Convert strings to float */
        strcpy( buff, totalP_str );
        f = atof( buff );
        TotalPowerUsed = f;
        strcpy( buff, nightlyP_str );
        f = atof( buff );
        NightlyPowerUsed = f;
    }

    /* Prepare log message and write it to log file */
    if (fp == NULL) {
        sprintf( buff, " INFO: Using power counters start values: Total=%3.3f, Nightly=%3.3f",
        TotalPowerUsed, NightlyPowerUsed );
        } else {
        sprintf( buff, " INFO: Read power counters start values: Total=%3.3f, Nightly=%3.3f",
        TotalPowerUsed, NightlyPowerUsed );
    }
    log_message(LOG_FILE, buff);
}

int
GPIOExport(int pin)
{
    char buffer[BUFFER_MAX];
    ssize_t bytes_written;
    int fd;

    fd = open("/sys/class/gpio/export", O_WRONLY);
    if (-1 == fd) {
        log_message(LOG_FILE," Failed to open export for writing!");
        return(-1);
    }

    bytes_written = snprintf(buffer, BUFFER_MAX, "%d", pin);
    write(fd, buffer, bytes_written);
    close(fd);
    return(0);
}

int
GPIOUnexport(int pin)
{
    char buffer[BUFFER_MAX];
    ssize_t bytes_written;
    int fd;

    fd = open("/sys/class/gpio/unexport", O_WRONLY);
    if (-1 == fd) {
        log_message(LOG_FILE," Failed to open unexport for writing!");
        return(-1);
    }

    bytes_written = snprintf(buffer, BUFFER_MAX, "%d", pin);
    write(fd, buffer, bytes_written);
    close(fd);
    return(0);
}

int
GPIODirection(int pin, int dir)
{
    static const char s_directions_str[]  = "in\0out";

    char path[DIRECTION_MAX];
    int fd;

    snprintf(path, DIRECTION_MAX, "/sys/class/gpio/gpio%d/direction", pin);
    fd = open(path, O_WRONLY);
    if (-1 == fd) {
        log_message(LOG_FILE," Failed to open GPIO direction for writing!");
        return(-1);
    }

    if (-1 == write(fd, &s_directions_str[IN == dir ? 0 : 3], IN == dir ? 2 : 3)) {
        log_message(LOG_FILE," Failed to set direction!");
        return(-1);
    }

    close(fd);
    return(0);
}

int
GPIORead(int pin)
{
    char path[VALUE_MAX];
    char value_str[3];
    int fd;

    snprintf(path, VALUE_MAX, "/sys/class/gpio/gpio%d/value", pin);
    fd = open(path, O_RDONLY);
    if (-1 == fd) {
        log_message(LOG_FILE," Failed to open GPIO value for reading!");
        return(-1);
    }

    if (-1 == read(fd, value_str, 3)) {
        log_message(LOG_FILE," Failed to read GPIO value!");
        return(-1);
    }

    close(fd);

    return(atoi(value_str));
}

int
GPIOWrite(int pin, int value)
{
    static const char s_values_str[] = "01";

    char path[VALUE_MAX];
    int fd;

    snprintf(path, VALUE_MAX, "/sys/class/gpio/gpio%d/value", pin);
    fd = open(path, O_WRONLY);
    if (-1 == fd) {
        log_message(LOG_FILE," Failed to open GPIO value for writing!");
        return(-1);
    }

    if (1 != write(fd, &s_values_str[LOW == value ? 0 : 1], 1)) {
        log_message(LOG_FILE," Failed to write GPIO value!");
        return(-1);
    }

    close(fd);
    return(0);
}

/*
    Example output of a sensor file:

    pi@raspberrypi ~ $ time cat /sys/bus/w1/devices/28-041464764cff/w1_slave
    84 01 55 00 3f ff 3f 10 d7 : crc=d7 YES
    84 01 55 00 3f ff 3f 10 d7 t=24250

    real    0m0.834s
    user    0m0.000s
    sys     0m0.050s
*/

float
sensorRead(const char* sensor)
{
    char path[VALUE_MAX];
    char value_str[50];
    int fd;
    char *str = "84 01 55 00 3f ff 3f 10 d7 t=114250";
    const char *result = str;
    long int_temp = 0;
    float temp = -200;
    /* if having trouble - return -200 */

    /* try to open sensor file */
    snprintf(path, VALUE_MAX, "%s", sensor);
    fd = open(path, O_RDONLY);
    if (-1 == fd) {
        log_message(LOG_FILE," Error opening sensor file. Continuing.");
        return(temp);
    }

    /* read the first line of data */
    if (-1 == read(fd, value_str, 39)) {
        log_message(LOG_FILE," Error reading from sensor file. Continuing.");
        close(fd);
        return(temp);
    }

    /* throw the first line away */
    strncpy(value_str, " ", 48);

    /* read the second line into value_str */
    if (-1 == read(fd, value_str, 35)) {
        log_message(LOG_FILE," Error reading row 2 from sensor file. Continuing.");
        close(fd);
        return(temp);
    }

    /* close the file - we are done with it */
    close(fd);

    /* transform sensor data to float */
    if((result = strchr((char *)&value_str, '=')) != NULL) {
        /* increment result to avoid the '=' */
        ++result;
        int_temp = atol( result );
        temp = ((float)int_temp) / 1000;
    }

    /* return the read temperature */
    return(temp);
}

void
signal_handler(int sig)
{
    switch(sig) {
        case SIGUSR1:
        log_message(LOG_FILE, " INFO: Signal SIGUSR1 caught. Will re-read config file soon.");
        need_to_read_cfg = 1;
        break;
        case SIGUSR2:
        log_message(LOG_FILE, " INFO: Signal SIGUSR2 caught. Not implemented. Continuing.");
        break;
        case SIGHUP:
        log_message(LOG_FILE, " INFO: Signal SIGHUP caught. Not implemented. Continuing.");
        break;
        case SIGTERM:
        log_message(LOG_FILE, " INFO: Terminate signal caught. Stopping.");
        if ( ! DisableGPIOpins() ) {
            log_message(LOG_FILE, " WARNING: Errors disabling GPIO! Quitting anyway.");
            exit(4);
        }
        log_message(LOG_FILE," Exiting normally. Bye, bye!");
        exit(0);
        break;
    }
}

void
daemonize()
{
    int i, lfp;
    char str[10];

    if(getppid()==1) return; /* already a daemon */
    i=fork();
    if (i<0) { printf("fork error!\n"); exit(1); }/* fork error */
    if (i>0) exit(0); /* parent exits */
    /* child (daemon) continues */
    setsid(); /* obtain a new process group */
    for (i=getdtablesize();i>=0;--i) close(i); /* close all descriptors */
    i=open("/dev/null",O_RDWR); dup(i); dup(i); /* handle standart I/O */
    umask(022); /* set newly created file permissions */
    chdir(RUNNING_DIR); /* change running directory */
    lfp=open(LOCK_FILE,O_RDWR|O_CREAT,0644);
    if (lfp<0) exit(2); /* can not open */
    if (lockf(lfp,F_TLOCK,0)<0) exit(0); /* can not lock */
    /* first instance continues */
    sprintf(str,"%d\n",getpid());
    write(lfp,str,strlen(str)); /* record pid to lockfile */
    signal(SIGCHLD,SIG_IGN); /* ignore child */
    signal(SIGTSTP,SIG_IGN); /* ignore tty signals */
    signal(SIGTTOU,SIG_IGN);
    signal(SIGTTIN,SIG_IGN);
    signal(SIGUSR1,signal_handler); /* catch signal USR1 */
    signal(SIGUSR2,signal_handler); /* catch signal USR2 */
    signal(SIGHUP,signal_handler); /* catch hangup signal */
    signal(SIGTERM,signal_handler); /* catch kill signal */
}

/* the following 3 functions RETURN 0 ON ERROR! (its to make the program nice to read) */
short
EnableGPIOpins()
{
    if (-1 == GPIOExport(PUMP1)) return 0;
    if (-1 == GPIOExport(PUMP2)) return 0;
    if (-1 == GPIOExport(VALVE)) return 0;
    if (-1 == GPIOExport(HEAT))  return 0;
    if (-1 == GPIOExport(POWER)) return 0;
    return -1;
}

short
SetGPIODirection()
{
    if (-1 == GPIODirection(PUMP1, OUT)) return 0;
    if (-1 == GPIODirection(PUMP2, OUT)) return 0;
    if (-1 == GPIODirection(VALVE, OUT)) return 0;
    if (-1 == GPIODirection(HEAT, OUT))  return 0;
    if (-1 == GPIODirection(POWER, IN))  return 0;
    return -1;
}

short
DisableGPIOpins()
{
    if (-1 == GPIOUnexport(PUMP1)) return 0;
    if (-1 == GPIOUnexport(PUMP2)) return 0;
    if (-1 == GPIOUnexport(VALVE)) return 0;
    if (-1 == GPIOUnexport(HEAT))  return 0;
    if (-1 == GPIOUnexport(POWER)) return 0;
    return -1;
}

void
ReadSensors() {
    float new_val = 0;
    int i;
    char msg[100];

    for (i=0;i<TOTALSENSORS;i++) {
        new_val = sensorRead(sensor_paths[i]);
        if ( new_val != -200 ) {
            if (sensor_read_errors[i]) sensor_read_errors[i]--;
            if (just_started) { sensors[i+5] = new_val; sensors[i+1] = new_val; }
            if (new_val < (sensors[i+5]-(2*MAX_TEMP_DIFF))) {
                sprintf( msg, " WARNING: Counting %3.3f for sensor %d as BAD and using %3.3f.",\
                new_val, i+1, sensors[i+5] );
                log_message(LOG_FILE, msg);
                new_val = sensors[i+5];
                sensor_read_errors[i]++;
            }
            if (new_val > (sensors[i+5]+(2*MAX_TEMP_DIFF))) {
                sprintf( msg, " WARNING: Counting %3.3f for sensor %d as BAD and using %3.3f.",\
                new_val, i+1, sensors[i+5] );
                log_message(LOG_FILE, msg);
                new_val = sensors[i+5];
                sensor_read_errors[i]++;
            }
            if (new_val < (sensors[i+5]-MAX_TEMP_DIFF)) {
                sprintf( msg, " WARNING: Correcting LOW %3.3f for sensor %d with %3.3f.",\
                new_val, i+1, sensors[i+5]-MAX_TEMP_DIFF );
                log_message(LOG_FILE, msg);
                new_val = sensors[i+5]-MAX_TEMP_DIFF;
            }
            if (new_val > (sensors[i+5]+MAX_TEMP_DIFF)) {
                sprintf( msg, " WARNING: Correcting HIGH %3.3f for sensor %d with %3.3f.",\
                new_val, i+1, sensors[i+5]+MAX_TEMP_DIFF );
                log_message(LOG_FILE, msg);
                new_val = sensors[i+5]+MAX_TEMP_DIFF;
            }
            sensors[i+5] = sensors[i+1];
            sensors[i+1] = new_val;
        }
        else {
            sensor_read_errors[i]++;
            sprintf( msg, " WARNING: Sensor %d ReadSensors() errors++. Counter at %d.", i+1, sensor_read_errors[i] );
            log_message(LOG_FILE, msg);
        }
    }
    /* Allow for maximum of 6 consecutive 10 second intervals of missing sensor data
    on any of the sensors before quitting screaming... */
    for (i=0;i<TOTALSENSORS;i++) {
        if (sensor_read_errors[i]>=5)){
            /* log the errors, clean up and bail out */
            if ( ! DisableGPIOpins() ) {
                log_message(LOG_FILE, " ALARM: Too many sensor errors! GPIO disable failed. Halting!");
                exit(5);
            }
            log_message(LOG_FILE, " ALARM: Too many sensor read errors! Stopping.");
            exit(6);
        }
    }
}

/* Read GPIO pin POWER into CPowerByBattery (see top of file)
which should be 1 if the external power is generated by UPS */
void
ReadExternalPower() {
    CPowerByBatteryPrev = CPowerByBattery;
    CPowerByBattery = GPIORead(POWER);
}

/* Return non-zero value on critical condition found based on current data in sensors[] */
short
CriticalTempsFound() {
    if (Tkotel >= 72) return 1;
    if (TboilerHigh >= 70) return 3;
    return 0;
}

/* Function to make GPIO state equal to controls[] (see top of file) */
void
ControlStateToGPIO() {
    /* put state on GPIO pins */
    GPIOWrite( PUMP1, CPump1 );
    GPIOWrite( PUMP2, CPump2 );
    GPIOWrite( VALVE, CValve );
    GPIOWrite( HEAT,  CHeater );
}

void
write_log_start() {
    char start_log_text[80];

    log_message(LOG_FILE," INFO: solard "SOLARDVERSION" now starting up...");
    log_message(LOG_FILE," Running in "RUNNING_DIR", config file "CONFIG_FILE );
    log_message(LOG_FILE," PID written to "LOCK_FILE", writing CSV data to "DATA_FILE );
    log_message(LOG_FILE," writing table data for collectd to "TABLE_FILE );
    log_message(LOG_FILE," power used persistence file "POWER_FILE );
    sprintf( start_log_text, " powers: heater=%3.1f W, pump=%3.1f W, valve=%3.1f W, self=%3.1f W",\
    HEATERPPC*(6*60), PUMPPPC*(6*60), VALVEPPC*(6*60), SELFPPC*(6*60) );
    log_message(LOG_FILE, start_log_text );
}

/* Function to get current time and put the hour in current_timer_hour */
void
GetCurrentTime() {
    static char buff[80];
    time_t t;
    struct tm *t_struct;
    short adjusted = 0;
    unsigned short current_day_of_month = 0;

    t = time(NULL);
    t_struct = localtime( &t );
    strftime( buff, sizeof buff, "%H", t_struct );

    current_timer_hour = atoi( buff );

    /* adjust night tariff start and stop hours at program start and
    every day sometime between 8:00 and 9:00 */
    if ( ( just_started ) ||
    ((current_timer_hour == 8) && ((ProgramRunCycles % (6*60)) == 0)) ) {
        strftime( buff, sizeof buff, "%m", t_struct );
        current_month = atoi( buff );
        if ((current_month >= 4)&&(current_month <= 10)) {
            /* April through October - use NE from 23:00 till 6:59 */
            if (NEstart != 23) {
                adjusted = 1;
                NEstart = 23;
                NEstop  = 6;
            }
        }
        else {
            /* November through March - use NE from 22:00 till 5:59 */
            if (NEstart != 22) {
                adjusted = 1;
                NEstart = 22;
                NEstop  = 5;
            }
        }
        if (adjusted) {
            sprintf( buff, " INFO: adjusted night energy hours, start %.2hu:00,"\
            " stop %.2hu:59.", NEstart, NEstop );
            log_message(LOG_FILE, buff);
        }
        /* among other things - manage power used counters */
        strftime( buff, sizeof buff, "%e", t_struct );
        current_day_of_month = atoi( buff );
        if (current_day_of_month == solard_cfg.day_to_reset_Pcounters) {
            /* if it is the right day - print power usage in log and reset counters */
            sprintf( buff, " INFO: Power used last month: daily: %2.2f Wh, nightly: %2.2f Wh;",
            (TotalPowerUsed-NightlyPowerUsed), NightlyPowerUsed );
            log_message(LOG_FILE, buff);
            sprintf( buff, " INFO: total: %2.2f Wh. Power counters reset.", TotalPowerUsed );
            log_message(LOG_FILE, buff);
            TotalPowerUsed = 0;
            NightlyPowerUsed = 0;
        }
    }
}

short
BoilerHeatingNeeded() {
    if ( TboilerLow > ((float)solard_cfg.wanted_T + 1) ) return 0;
    if ( TboilerHigh < ((float)solard_cfg.wanted_T - 0.3) ) return 1;
    if ( (TboilerHigh < TboilerHighPrev) &&
    (TboilerHighPrev < (float)solard_cfg.wanted_T ) ) return 1;
    return 0;
}

void
LogData(short HM) {
    static char data[280];
    /* Log data like so:
        Time(by log function), TKOTEL,TSOLAR,TBOILERL,TBOILERH, BOILERTEMPWANTED,HM,
    PUMP1,PUMP2,VALVE,HEAT,POWERBYBATTERY, WATTSUSED,WATTSUSEDNIGHTTARIFF */
    sprintf( data, ", %3.3f,%3.3f,%3.3f,%3.3f, %2d,%2d, %d,%d,%d,%d,%d, %3.3f,%3.3f",\
    Tkotel, Tkolektor, TboilerLow, TboilerHigh, solard_cfg.wanted_T, HM, CPump1,\
    CPump2, CValve, CHeater, CPowerByBattery, TotalPowerUsed, NightlyPowerUsed );
    log_message(DATA_FILE, data);

    sprintf( data, ",Temp1,%3.3f\n_,Temp2,%3.3f\n_,Temp3,%3.3f\n_,Temp4,%3.3f\n_"\
    ",Pump1,%d\n_,Pump2,%d\n_,Valve,%d\n_,Heater,%d\n_,PoweredByBattery,%d\n_"\
    ",TempWanted,%d\n_,ElectricityUsed,%lu\n_,ElectricityUsedNT,%lu",\
    Tkotel, Tkolektor, TboilerHigh, TboilerLow, CPump1, CPump2,\
    CValve, CHeater, CPowerByBattery, solard_cfg.wanted_T,\
    (long)TotalPowerUsed, (long)NightlyPowerUsed );
    log_msg_ovr(TABLE_FILE, data);
}

short
SelectIdleMode() {
    short ModeSelected = 0;
    short wantP1on = 0;
    short wantP2on = 0;
    short wantVon = 0;
    /* If furnace is cold - turn pump on to keep it from freezing */
    if (Tkotel < 8.9) wantP1on = 1;
    /* If solar is cold - turn pump on to keep it from freezing */
    if (Tkolektor < 8.9) wantP2on = 1;
    /* Furnace is above 52 - at these temps always run the pump */
    if (Tkotel > 52) wantP1on = 1;
    /* Furnace is above 45 and rising - turn pump on */
    if ((Tkotel > 44.9)&&(Tkotel > TkotelPrev+0.06)) wantP1on = 1;
    /* Furnace is above 36 and rising slowly - turn pump on */
    if ((Tkotel > 35.9)&&(Tkotel > (TkotelPrev+0.12))) wantP1on = 1;
    /* Furnace is above 24 and rising QUICKLY - turn pump on to limit furnace thermal shock */
    if ((Tkotel > 21.9)&&(Tkotel > (TkotelPrev+0.18))) wantP1on = 1;
    /* Solar has heat in excess - rise boiler temp to 62 C so expensive sources
    are not used later on during the day */
    if ((Tkolektor > (TboilerLow+14.9))&&(TboilerHigh < 62)) wantP2on = 1;
    /* Keep solar pump on while temp diff is 5 C or more */
    if ((CPump2) && (Tkolektor >= (TboilerLow+5))) wantP2on = 1;
    /* Try to heat the house by taking heat from boiler but leave at least 5 C extra on
    top of the wanted temp - turn furnace pump on and open the valve */
    if ( (solard_cfg.mode==2) && /* 2=AUTO+HEAT HOUSE BY SOLAR; */
    (TboilerHigh > ((float)solard_cfg.wanted_T + 5)) && (TboilerLow > Tkotel) ) {
        wantP1on = 1;
        wantVon = 1;
    }
    /* Furnace has heat in excess - open the valve so boiler can build up
    heat now and probably save on electricity use later on */
    if ((TboilerHigh > (float)solard_cfg.wanted_T)&&(Tkotel > (TboilerHigh+9.9))) {
        wantVon = 1;
        /* And if valve has been open for 2 minutes - turn furnace pump on */
        if (CValve && (SCValve > 9)) wantP1on = 1;
    }
    /* Try to keep Grundfoss UPS2 pump dandy - turn it on every 48 hours */
    if ( (!CPump1) && (SCPump1 > (6*60*48)) ) wantP1on = 1;
    /* If solar pump has been off for 60 minutes during day time - turn it on for a while,
    to circulate fluid */
    if ( (!CPump2) && (SCPump2 > (6*60)) &&
    (current_timer_hour >= 11) && (current_timer_hour <= 16)) wantP2on = 1;
    if (solard_cfg.keep_pump1_on) wantP1on = 1;
    /* If solar is too hot - do not damage other equipment with the hot water */
    if (Tkolektor > 85) wantP2on = 0;

    if ( wantP1on ) ModeSelected |= 1;
    if ( wantP2on ) ModeSelected |= 2;
    if ( wantVon )  ModeSelected |= 4;
    return ModeSelected;
}

short
SelectHeatingMode() {
    short ModeSelected = 0;
    short wantP1on = 0;
    short wantP2on = 0;
    short wantVon = 0;
    short wantHon = 0;

    /* First get what the idle routine would do: */
    ModeSelected = SelectIdleMode();

    /* Then add to it main Select()'s stuff: */
    if ((Tkolektor > (TboilerLow + 14.9))&&(Tkolektor > Tkotel)) {
        /* To enable solar heating, ECT temp must be at least 15 C higher than the boiler */
        wantP2on = 1;
    }
    else {
        /* Not enough heat in the solar collector; check other sources of heat */
        if (Tkotel > (TboilerHigh + 9.9)) {
            /* The furnace is hot enough - use it */
            wantVon = 1;
            /* And if valve has been open for 2 minutes - turn furnace pump on */
            if (CValve &&(SCValve > 13)) wantP1on = 1;
        }
        else {
            /* All is cold - use electric heater if possible */
            /* FIXME For now - only turn heater on if valve is fully closed,
            because it runs with at least one pump */
            if (!CValve && (SCValve > 15)) wantHon = 1;
        }
    }

    if ( wantP1on ) ModeSelected |= 1;
    if ( wantP2on ) ModeSelected |= 2;
    if ( wantVon )  ModeSelected |= 4;
    if ( wantHon )  ModeSelected |= 8;
    return ModeSelected;
}

void TurnPump1Off()  { if (CPump1 && !CValve && (SCPump1 > 5) && (SCValve > 5))
{ CPump1 = 0; SCPump1 = 0; } }
void TurnPump1On()   { if (!CPump1) { CPump1 = 1; SCPump1 = 0; } }
void TurnPump2Off()  { if (CPump2 && (SCPump2 > 2)) { CPump2  = 0; SCPump2 = 0; } }
void TurnPump2On()   { if (solard_cfg.use_pump2 && (!CPump2) && (SCPump2 > 2)) { CPump2  = 1; SCPump2 = 0; } }
void TurnValveOff()  { if (CValve && (SCValve > 23)) { CValve  = 0; SCValve = 0; } }
void TurnValveOn()   { if (!CValve && (SCValve > 5)) { CValve  = 1; SCValve = 0; } }
void TurnHeaterOff() { if (CHeater && (SCHeater > 11)) { CHeater = 0; SCHeater = 0; } }
void TurnHeaterOn()  { if ((!CHeater) && (SCHeater > 29)) { CHeater = 1; SCHeater = 0; } }

void
RequestElectricHeat() {
    /* Do the check with config to see if its OK to use electric heater,
    for example: if its on "night tariff" - switch it on */
    if ( solard_cfg.use_electric_start_hour > solard_cfg.use_electric_stop_hour ) {
        /* heater allowed like from 23:00(23) to 05:00(4) - so use OR */
        if ( (current_timer_hour >= solard_cfg.use_electric_start_hour) ||
        (current_timer_hour < solard_cfg.use_electric_stop_hour) ) {
            /* allowed time - if heater is off - turn it on */
            TurnHeaterOn();
        }
    }
    else {
        /* heater allowed like from 05:00(5) to 07:00(6) - so use AND */
        if ( (current_timer_hour >= solard_cfg.use_electric_start_hour) &&
        (current_timer_hour < solard_cfg.use_electric_stop_hour) ) {
            /* allowed time - if heater is off - turn it on */
            TurnHeaterOn();
        }
    }
}

void
ActivateHeatingMode(const short HeatMode) {
    char current_state = 0;
    char new_state = 0;

    /* calculate current state */
    if ( CPump1 ) current_state |= 1;
    if ( CPump2 ) current_state |= 2;
    if ( CValve ) current_state |= 4;
    if ( CHeater ) current_state |= 8;
    /* make changes as needed */
    /* HeatMode's bits describe the peripherals desired state:
        bit 0  (1) - pump 1
        bit 1  (2) - pump 2
        bit 2  (4) - valve
        bit 3  (8) - heater wanted
    bit 4 (16) - heater forced */
    if (HeatMode & 1)  { TurnPump1On(); } else { TurnPump1Off(); }
    if (HeatMode & 2)  { TurnPump2On(); } else { TurnPump2Off(); }
    if (HeatMode & 4)  { TurnValveOn(); } else { TurnValveOff(); }
    if (HeatMode & 8)  { RequestElectricHeat(); }
    if (HeatMode & 16) { TurnHeaterOn(); }
    if ( !(HeatMode & 24) ) { TurnHeaterOff(); }
    SCPump1++;
    SCPump2++;
    SCValve++;
    SCHeater++;

    /* Calculate total and night tariff electrical power used here: */
    if ( CHeater ) {
        TotalPowerUsed += HEATERPPC;
        if ( (current_timer_hour <= NEstop) || (current_timer_hour >= NEstart) ) { NightlyPowerUsed += HEATERPPC; }
    }
    if ( CPump1 ) {
        TotalPowerUsed += PUMPPPC;
        if ( (current_timer_hour <= NEstop) || (current_timer_hour >= NEstart) ) { NightlyPowerUsed += PUMPPPC; }
    }
    if ( CPump2 ) {
        TotalPowerUsed += PUMPPPC;
        if ( (current_timer_hour <= NEstop) || (current_timer_hour >= NEstart) ) { NightlyPowerUsed += PUMPPPC; }
    }
    if ( CValve ) {
        TotalPowerUsed += VALVEPPC;
        if ( (current_timer_hour <= NEstop) || (current_timer_hour >= NEstart) ) { NightlyPowerUsed += VALVEPPC; }
    }
    TotalPowerUsed += SELFPPC;
    if ( (current_timer_hour <= NEstop) || (current_timer_hour >= NEstart) ) { NightlyPowerUsed += SELFPPC; }

    /* calculate desired new state */
    if ( CPump1 ) new_state |= 1;
    if ( CPump2 ) new_state |= 2;
    if ( CValve ) new_state |= 4;
    if ( CHeater ) new_state |= 8;
    /* if current state and new state are different... */
    if ( current_state != new_state ) {
        /* then put state on GPIO pins - this prevents lots of toggling at every 10s decision */
        ControlStateToGPIO();
    }
}

void
AdjustHeatingModeForBatteryPower(short HM) {
    /* Check for power source switch */
    if ( CPowerByBattery != CPowerByBatteryPrev ) {
        /* If we just switched to battery.. */
        if ( CPowerByBattery ) {
            log_message(LOG_FILE," WARNING: Switch to BATTERY POWER detected.");
        }
        else {
            log_message(LOG_FILE," INFO: Powered by GRID now.");
        }
    }
    if ( CPowerByBattery ) {
        /* When battery powered - electric heater does not work; do not try it */
        HM &= ~(1 << 4);
        HM &= ~(1 << 5);
        /* enable quick heater turn off */
        if (CHeater && (SCHeater < 12)) { SCHeater = 12; }
    }
}

int
main(int argc, char *argv[])
{
    /* set iter to its max value - makes sure we get a clock reading upon start */
    short iter = 30;
    short iter_P = 0;
    short AlarmRaised = 0;
    short HeatingMode = 0;
    struct timeval tvalBefore, tvalAfter;

    SetDefaultCfg();

    /* before main work starts - try to open the log files to write a new line
    ...and SCREAM if there is trouble! */
    if (log_message(LOG_FILE," ***\n")) {
        printf(" Cannot open the mandatory "LOG_FILE" file needed for operation!\n");
        exit(3);
    }
    if (log_message(DATA_FILE," ***\n")) {
        printf(" Cannot open the mandatory "DATA_FILE" file needed for operation!\n");
        exit(3);
    }
    if (log_message(TABLE_FILE," ***\n")) {
        printf(" Cannot open the mandatory "TABLE_FILE" file needed for operation!\n");
        exit(3);
    }

    daemonize();

    /* Enable GPIO pins */
    if ( ! EnableGPIOpins() ) {
        log_message(LOG_FILE," ALARM: Cannot enable GPIO! Aborting run.");
        return(1);
    }

    /* Set GPIO directions */
    if ( ! SetGPIODirection() ) {
        log_message(LOG_FILE," ALARM: Cannot set GPIO direction! Aborting run.");
        return(2);
    }

    write_log_start();

    parse_config();

    just_started = 1;
    TotalPowerUsed = 0;
    NightlyPowerUsed = 0;

    ReadPersistentPower();

    do {
        /* Do all the important stuff... */
        if ( gettimeofday( &tvalBefore, NULL ) ) {
            log_message(LOG_FILE," WARNING: error getting tvalBefore...");
        }
        /* get the current hour every 5 minutes for electric heater schedule */
        if ( iter == 30 ) {
            iter = 0;
            GetCurrentTime();
            /* and increase counter controlling writing out persitent power use data */
            iter_P++;
            if ( iter_P == 2) {
                iter_P = 0;
                WritePersistentPower();
            }
        }
        iter++;
        ReadSensors();
        ReadExternalPower();
        /* do what "mode" from CFG files says - watch the LOG file to see used values */
        switch (solard_cfg.mode) {
            default:
            case 0: /* 0=ALL OFF */
            HeatingMode = 0;
            break;
            case 1: /* 1=AUTO - tries to reach desired water temp efficiently */
            case 2: /* 2=AUTO+HEAT HOUSE BY SOLAR - mode taken into account by SelectIdle() */
            if ( CriticalTempsFound() ) {
                /* ActivateEmergencyHeatTransfer(); */
                /* Set HeatingMode bits for both pumps and valve */
                HeatingMode = 7;
                if ( !AlarmRaised ) {
                    log_message(LOG_FILE," ALARM: Activating emergency cooling!");
                    AlarmRaised = 1;
                }
            }
            else {
                if ( AlarmRaised ) {
                    log_message(LOG_FILE," INFO: Critical condition resolved. Running normally.");
                    AlarmRaised = 0;
                }
                if (BoilerHeatingNeeded()) {
                    HeatingMode = SelectHeatingMode();
                    } else {
                    /* No heating needed - decide how to idle */
                    HeatingMode = SelectIdleMode();
                    HeatingMode |= 32;
                }
            }
            break;
            case 3: /* 3=MANUAL PUMP1 ONLY - only furnace pump ON */
            HeatingMode = 1;
            break;
            case 4: /* 4=MANUAL PUMP2 ONLY - only solar pump ON */
            HeatingMode = 2;
            break;
            case 5: /* 5=MANUAL HEATER ONLY - set THERMOSTAT CORRECTLY!!! */
            HeatingMode = 16;
            break;
            case 6: /* 6=MANAUL PUMP1+HEATER - furnace pump and heater power ON */
            HeatingMode = 17;
            break;
            case 7: /* 7=AUTO ELECTICAL HEATER ONLY - this one obeys start/stop hours */
            if (BoilerHeatingNeeded()) {
                HeatingMode = 8;
                } else {
                HeatingMode = 32;
            }
            break;
            case 8: /* 8=AUTO ELECTICAL HEATER ONLY, DOES NOT CARE ABOUT SCHEDULE !!! */
            if (BoilerHeatingNeeded()) {
                HeatingMode = 16;
                } else {
                HeatingMode = 32;
            }
            break;
        }
        AdjustHeatingModeForBatteryPower(HeatingMode);
        ActivateHeatingMode(HeatingMode);
        LogData(HeatingMode);
        ProgramRunCycles++;
        if ( just_started ) { just_started = 0; }
        if ( need_to_read_cfg ) {
            need_to_read_cfg = 0;
            just_started = 1;
            parse_config();
        }
        if ( gettimeofday( &tvalAfter, NULL ) ) {
            log_message(LOG_FILE," WARNING: error getting tvalAfter...");
            sleep( 4 );
        }
        else {
            /* if we have not run for 5 minutes or time is skewed (ex. daylitght saving) */
            if ( (ProgramRunCycles <= 30) || ((tvalAfter.tv_sec - tvalBefore.tv_sec) >= 15) ) {
                /* use hardcoded sleep() */
                sleep( 4 );
            }
            else {
                /* otherwise we have valid time data - so calculate exact sleep time
                so period between active operations is bang on 10 seconds */
                usleep(10000000 - (((tvalAfter.tv_sec - tvalBefore.tv_sec)*1000000L \
                + tvalAfter.tv_usec) - tvalBefore.tv_usec));
            }
        }
    } while (1);

    /* Disable GPIO pins */
    if ( ! DisableGPIOpins() ) {
        log_message(LOG_FILE," Cannot disable GPIO on exit!");
        return(4);
    }

    return(0);
}

/* EOF */
