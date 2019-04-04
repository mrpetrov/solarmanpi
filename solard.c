/*
* solard.c
*
* Raspberry Pi solar manager which uses 1wire and GPIO.
* Plamen Petrov
*
* solard is Plamen's custom solar controller, based on the Raspberry Pi 2.
* Data is gathered and logged every 10 seconds from 4 DS18B20 waterproof sensors,
* 4 relays are controlled via GPIO, and a GPIO pin is read to note current
* power source: grid or battery backed UPS.
* Log data is in CSV format, to be picked up by some sort of data collection/graphing
* tool, like collectd or similar. There is also JSON file more suitable for sending data
* to data collection software like mqqt/emoncms.
* The daemon is controlled via its configuration file, which solard can be told to
* re-read and parse while running to change config in flight. This is done by
* sending SIGUSR1 signal to the daemon process. The event is noted in the log file.
* The logfile itself can be "grep"-ed for "ALARM" and "INFO" to catch and notify
* of notable events, recorded by the daemon.
*/

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
#define JSON_FILE	"/run/shm/solard_current_json"
#define CFG_TABLE_FILE  "/run/shm/solard_cur_cfg"
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
#define GPIO_PIN_PUMP1           17 /* P1-11 */
#define GPIO_PIN_PUMP2           18 /* P1-12 */
#define GPIO_PIN_VALVE           27 /* P1-13 */
/*                                     P1-14 == GROUND */
#define GPIO_PIN_EL_HEATER       22 /* P1-15 */
#define GPIO_PIN_FURNACE         23 /* P1-16 NOTE: Reserved for future implementation */
#define GPIO_PIN_UPS_POWERED     25 /* P1-22 */

/* Maximum difference allowed for data received from sensors between reads, C */
#define MAX_TEMP_DIFF        7

/* Number of all sensors to be used by the system */
#define TOTALSENSORS         4

/* Define the DS18B20 sensors paths to read temps from */
#define SENSOR1 "/sys/bus/w1/devices/28-041464764cff/w1_slave"
#define SENSOR2 "/sys/bus/w1/devices/28-041464403bff/w1_slave"
#define SENSOR3 "/sys/bus/w1/devices/28-04146448f3ff/w1_slave"
#define SENSOR4 "/sys/bus/w1/devices/28-0214608d40ff/w1_slave"

char *sensor_paths[] = { SENSOR1, SENSOR2, SENSOR3, SENSOR4 };

/*  var to keep track of read errors, so if a threshold is reached - the
    program can safely shut down everything, send notification and bail out;
    initialised with borderline value to trigger immediately on errors during
    start-up; the program logic tolerates 1 minute of missing sensor data
*/
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

float nightEnergyTemp;

/* solard keeps track of total and night tariff watt-hours electrical power used */
/* night tariff is between 23:00 and 06:00 */
/* constants of Watt-hours of electricity used per 10 secs */
#define   HEATERPPC         8.340
#define   PUMP1PPC          0.135
#define   PUMP2PPC          0.021
#define   VALVEPPC          0.006
#define   SELFPPC           0.022
/* my boiler uses 3kW per hour, so this is 0,00834 kWh per 10 seconds */
/* this in Wh per 10 seconds is 8.34 W */
/* pump 1 (furnace) runs at 48 W setting, pump 2 (solar) - 7 W */

/* NightEnergy (NE) start and end hours variables - get recalculated every day */
unsigned short NEstart = 20;
unsigned short NEstop  = 11;

/* Nubmer of cycles (circa 10 seconds each) that the program has run */
unsigned long ProgramRunCycles  = 0;

/* timers - current hour and month vars - used in keeping things up to date */
unsigned short current_timer_hour = 0;
unsigned short current_month = 0;

/* a var to be non-zero if it is winter time - so furnace should not be allowed to go too cold */
unsigned short now_is_winter = 0;

/* array storing the hour at wich to make the solar pump daily run for each month */
unsigned short pump_start_hour_for[13] = { 11, 14, 13, 12, 11, 10, 9, 9, 10, 11, 12, 13, 14 };

struct cfg_struct
{
    char    mode_str[MAXLEN];
    int     mode;
    char    wanted_T_str[MAXLEN];
    int     wanted_T;
    char    use_electric_heater_night_str[MAXLEN];
    int     use_electric_heater_night;
    char    use_electric_heater_day_str[MAXLEN];
    int     use_electric_heater_day;
    char    pump1_always_on_str[MAXLEN];
    int     pump1_always_on;
    char    use_pump1_str[MAXLEN];
    int     use_pump1;
    char    use_pump2_str[MAXLEN];
    int     use_pump2;
    char    day_to_reset_Pcounters_str[MAXLEN];
    int     day_to_reset_Pcounters;
    char    night_boost_str[MAXLEN];
    int     night_boost;
    char    abs_max_str[MAXLEN];
    int     abs_max;
}
cfg_struct;

struct cfg_struct cfg;

short need_to_read_cfg = 0;

short just_started = 0;

/* FORWARD DECLARATIONS so functions can be used in preceding ones */
short
DisableGPIOpins();
/* end of forward-declared functions */

void
rangecheck_mode( int m )
{
    if (m < 0) m = 0;
    if (m > 8) m = 0;
}

void
rangecheck_wanted_temp( int temp )
{
    if (temp < 10) temp = 10;
    if (temp > 50) temp = 50;
}

void
rangecheck_abs_max_temp( int t )
{
    if (t < 30) t = 30;
    if (t > 65) t = 65;
}

void
rangecheck_day_of_month( int d )
{
    if (d < 1) d = 1;
    if (d > 28) d = 28;
}

void
SetDefaultCfg() {
    strcpy( cfg.mode_str, "1");
    cfg.mode = 1;
    strcpy( cfg.wanted_T_str, "40");
    cfg.wanted_T = 40;
    strcpy( cfg.use_electric_heater_night_str, "1");
    cfg.use_electric_heater_night = 1;
    strcpy( cfg.use_electric_heater_day_str, "1");
    cfg.use_electric_heater_day = 1;
    strcpy( cfg.pump1_always_on_str, "0");
    cfg.pump1_always_on = 0;
    strcpy( cfg.use_pump1_str, "1");
    cfg.use_pump1 = 1;
    strcpy( cfg.use_pump2_str, "1");
    cfg.use_pump2 = 1;
    strcpy( cfg.day_to_reset_Pcounters_str, "4");
    cfg.day_to_reset_Pcounters = 4;
    strcpy( cfg.night_boost_str, "0");
    cfg.night_boost = 0;
    strcpy( cfg.abs_max_str, "47");
    cfg.abs_max = 47;

    nightEnergyTemp = 0;
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
    sprintf( file_string, "%s %s", timestamp, message );
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

/* this version of the logging function destroys the opened file contents, no timestamp and new line */
void
log_msg_cln(char *filename, char *message) {
    FILE *logfile;
    char file_string[300];

    sprintf( file_string, "%s", message );
    logfile = fopen( filename, "w" );
    if ( !logfile ) return;
    fprintf( logfile, "%s", file_string );
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
        log_message(LOG_FILE,"WARNING: Failed to open "CONFIG_FILE" file for reading!");
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
            strncpy (cfg.mode_str, value, MAXLEN);
            else if (strcmp(name, "wanted_T")==0)
            strncpy (cfg.wanted_T_str, value, MAXLEN);
            else if (strcmp(name, "use_electric_heater_night")==0)
            strncpy (cfg.use_electric_heater_night_str, value, MAXLEN);
            else if (strcmp(name, "use_electric_heater_day")==0)
            strncpy (cfg.use_electric_heater_day_str, value, MAXLEN);
            else if (strcmp(name, "pump1_always_on")==0)
            strncpy (cfg.pump1_always_on_str, value, MAXLEN);
            else if (strcmp(name, "use_pump1")==0)
            strncpy (cfg.use_pump1_str, value, MAXLEN);
            else if (strcmp(name, "use_pump2")==0)
            strncpy (cfg.use_pump2_str, value, MAXLEN);
            else if (strcmp(name, "day_to_reset_Pcounters")==0)
            strncpy (cfg.day_to_reset_Pcounters_str, value, MAXLEN);
            else if (strcmp(name, "night_boost")==0)
            strncpy (cfg.night_boost_str, value, MAXLEN);
            else if (strcmp(name, "abs_max")==0)
            strncpy (cfg.abs_max_str, value, MAXLEN);
        }
        /* Close file */
        fclose (fp);
    }

    /* Convert strings to int */
    strcpy( buff, cfg.mode_str );
    i = atoi( buff );
    cfg.mode = i;
    rangecheck_mode( cfg.mode );
    strcpy( buff, cfg.wanted_T_str );
    i = atoi( buff );
    if ( i ) cfg.wanted_T = i;
    rangecheck_wanted_temp( cfg.wanted_T );
    strcpy( buff, cfg.use_electric_heater_night_str );
    i = atoi( buff );
    cfg.use_electric_heater_night = i;
    /* ^ no need for range check - 0 is OFF, non-zero is ON */
    strcpy( buff, cfg.use_electric_heater_day_str );
    i = atoi( buff );
    cfg.use_electric_heater_day = i;
    /* ^ no need for range check - 0 is OFF, non-zero is ON */
    strcpy( buff, cfg.pump1_always_on_str );
    i = atoi( buff );
    cfg.pump1_always_on = i;
    /* ^ no need for range check - 0 is OFF, non-zero is ON */
    strcpy( buff, cfg.use_pump1_str );
    i = atoi( buff );
    cfg.use_pump1 = i;
    /* ^ no need for range check - 0 is OFF, non-zero is ON */
    strcpy( buff, cfg.use_pump2_str );
    i = atoi( buff );
    cfg.use_pump2 = i;
    /* ^ no need for range check - 0 is OFF, non-zero is ON */
    strcpy( buff, cfg.day_to_reset_Pcounters_str );
    i = atoi( buff );
    if ( i ) cfg.day_to_reset_Pcounters = i;
    rangecheck_day_of_month( cfg.day_to_reset_Pcounters );
    strcpy( buff, cfg.night_boost_str );
    i = atoi( buff );
    cfg.night_boost = i;
    /* ^ no need for range check - 0 is OFF, non-zero is ON */
    strcpy( buff, cfg.abs_max_str );
    i = atoi( buff );
    if (i < (cfg.wanted_T+3)) { i = cfg.wanted_T+3; }
    cfg.abs_max = i;
    rangecheck_abs_max_temp( cfg.abs_max );

    /* Prepare log message part 1 and write it to log file */
    if (fp == NULL) {
        sprintf( buff, "INFO: Using values: Mode=%d, wanted temp=%d, el. heater: night=%d, day=%d,",\
        cfg.mode, cfg.wanted_T, cfg.use_electric_heater_night, cfg.use_electric_heater_day );
        } else {
        sprintf( buff, "INFO: Read CFG file: Mode=%d, wanted temp=%d, el. heater: night=%d, day=%d,",\
        cfg.mode, cfg.wanted_T, cfg.use_electric_heater_night, cfg.use_electric_heater_day );
    }
    log_message(LOG_FILE, buff);
    /* Prepare log message part 2 and write it to log file */
    sprintf( buff, "INFO: furnace pump always on=%d, use furnace pump=%d, use solar pump=%d, reset P counters day=%d, "\
    "night boiler boost=%d, absMAX=%d", cfg.pump1_always_on, cfg.use_pump1, cfg.use_pump2,\
    cfg.day_to_reset_Pcounters, cfg.night_boost, cfg.abs_max );
    log_message(LOG_FILE, buff);
	
    /* stuff for after parsing config file: */
    /* calculate maximum possible temp for use in night_boost case */
    nightEnergyTemp = ((float)cfg.wanted_T + 12);
    if (nightEnergyTemp > (float)cfg.abs_max) { nightEnergyTemp = (float)cfg.abs_max; }
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
    fprintf( logfile, "total=%6.3f\n", TotalPowerUsed );
    fprintf( logfile, "nightly=%6.3f\n", NightlyPowerUsed );
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
        log_message(LOG_FILE,"WARNING: Failed to open "POWER_FILE" file for reading!");
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
        log_message(LOG_FILE, "Creating missing power persistence data file...");
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
        sprintf( buff, "INFO: Using power counters start values: Total=%6.3f, Nightly=%6.3f",
        TotalPowerUsed, NightlyPowerUsed );
        } else {
        sprintf( buff, "INFO: Read power counters start values: Total=%6.3f, Nightly=%6.3f",
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
        log_message(LOG_FILE,"Failed to open export for writing!");
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
        log_message(LOG_FILE,"Failed to open unexport for writing!");
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
        log_message(LOG_FILE,"Failed to open GPIO direction for writing!");
        return(-1);
    }

    if (-1 == write(fd, &s_directions_str[IN == dir ? 0 : 3], IN == dir ? 2 : 3)) {
        log_message(LOG_FILE,"Failed to set direction!");
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
        log_message(LOG_FILE,"Failed to open GPIO value for reading!");
        return(-1);
    }

    if (-1 == read(fd, value_str, 3)) {
        log_message(LOG_FILE,"Failed to read GPIO value!");
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
        log_message(LOG_FILE,"Failed to open GPIO value for writing!");
        return(-1);
    }

    if (1 != write(fd, &s_values_str[LOW == value ? 0 : 1], 1)) {
        log_message(LOG_FILE,"Failed to write GPIO value!");
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
        log_message(LOG_FILE,"Error opening sensor file. Continuing.");
        return(temp);
    }

    /* read the first line of data */
    if (-1 == read(fd, value_str, 39)) {
        log_message(LOG_FILE,"Error reading from sensor file. Continuing.");
        close(fd);
        return(temp);
    }

    /* throw the first line away */
    strncpy(value_str, " ", 48);

    /* read the second line into value_str */
    if (-1 == read(fd, value_str, 35)) {
        log_message(LOG_FILE,"Error reading row 2 from sensor file. Continuing.");
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
        log_message(LOG_FILE, "INFO: Signal SIGUSR1 caught. Will re-read config file soon.");
        need_to_read_cfg = 1;
        break;
        case SIGUSR2:
        log_message(LOG_FILE, "INFO: Signal SIGUSR2 caught. Not implemented. Continuing.");
        break;
        case SIGHUP:
        log_message(LOG_FILE, "INFO: Signal SIGHUP caught. Not implemented. Continuing.");
        break;
        case SIGTERM:
        log_message(LOG_FILE, "INFO: Terminate signal caught. Stopping.");
        WritePersistentPower();
        if ( ! DisableGPIOpins() ) {
            log_message(LOG_FILE, "WARNING: Errors disabling GPIO! Quitting anyway.");
            exit(4);
        }
        log_message(LOG_FILE,"Exiting normally. Bye, bye!");
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
    if (-1 == GPIOExport(GPIO_PIN_PUMP1)) return 0;
    if (-1 == GPIOExport(GPIO_PIN_PUMP2)) return 0;
    if (-1 == GPIOExport(GPIO_PIN_VALVE)) return 0;
    if (-1 == GPIOExport(GPIO_PIN_EL_HEATER))  return 0;
    if (-1 == GPIOExport(GPIO_PIN_UPS_POWERED)) return 0;
    return -1;
}

short
SetGPIODirection()
{
    /* output pins */
    if (-1 == GPIODirection(GPIO_PIN_PUMP1, OUT)) return 0;
    if (-1 == GPIODirection(GPIO_PIN_PUMP2, OUT)) return 0;
    if (-1 == GPIODirection(GPIO_PIN_VALVE, OUT)) return 0;
    if (-1 == GPIODirection(GPIO_PIN_EL_HEATER, OUT))  return 0;
    /* input pins */
    if (-1 == GPIODirection(GPIO_PIN_UPS_POWERED, IN))  return 0;
    return -1;
}

short
DisableGPIOpins()
{
    if (-1 == GPIOUnexport(GPIO_PIN_PUMP1)) return 0;
    if (-1 == GPIOUnexport(GPIO_PIN_PUMP2)) return 0;
    if (-1 == GPIOUnexport(GPIO_PIN_VALVE)) return 0;
    if (-1 == GPIOUnexport(GPIO_PIN_EL_HEATER))  return 0;
    if (-1 == GPIOUnexport(GPIO_PIN_UPS_POWERED)) return 0;
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
                sprintf( msg, "WARNING: Counting %6.3f for sensor %d as BAD and using %6.3f.",\
                new_val, i+1, sensors[i+5] );
                log_message(LOG_FILE, msg);
                new_val = sensors[i+5];
                sensor_read_errors[i]++;
            }
            if (new_val > (sensors[i+5]+(2*MAX_TEMP_DIFF))) {
                sprintf( msg, "WARNING: Counting %6.3f for sensor %d as BAD and using %6.3f.",\
                new_val, i+1, sensors[i+5] );
                log_message(LOG_FILE, msg);
                new_val = sensors[i+5];
                sensor_read_errors[i]++;
            }
            if (new_val < (sensors[i+5]-MAX_TEMP_DIFF)) {
                sprintf( msg, "WARNING: Correcting LOW %6.3f for sensor %d with %6.3f.",\
                new_val, i+1, sensors[i+5]-MAX_TEMP_DIFF );
                log_message(LOG_FILE, msg);
                new_val = sensors[i+5]-MAX_TEMP_DIFF;
            }
            if (new_val > (sensors[i+5]+MAX_TEMP_DIFF)) {
                sprintf( msg, "WARNING: Correcting HIGH %6.3f for sensor %d with %6.3f.",\
                new_val, i+1, sensors[i+5]+MAX_TEMP_DIFF );
                log_message(LOG_FILE, msg);
                new_val = sensors[i+5]+MAX_TEMP_DIFF;
            }
            sensors[i+5] = sensors[i+1];
            sensors[i+1] = new_val;
        }
        else {
            sensor_read_errors[i]++;
            sprintf( msg, "WARNING: Sensor %d ReadSensors() errors++. Counter at %d.", i+1, sensor_read_errors[i] );
            log_message(LOG_FILE, msg);
        }
    }
    /* Allow for maximum of 6 consecutive 10 second intervals of missing sensor data
    on any of the sensors before quitting screaming... */
    for (i=0;i<TOTALSENSORS;i++) {
        if (sensor_read_errors[i]>5) {
            /* log the errors, clean up and bail out */
            if ( ! DisableGPIOpins() ) {
                log_message(LOG_FILE, "ALARM: Too many sensor errors! GPIO disable failed. Halting!");
                exit(5);
            }
            log_message(LOG_FILE, "ALARM: Too many sensor read errors! Stopping.");
            exit(6);
        }
    }
}

/* Read GPIO_PIN_UPS_POWERED into CPowerByBattery (see top of file)
which should be 1 if the external power is generated by UPS */
void
ReadExternalPower() {
    CPowerByBatteryPrev = CPowerByBattery;
    CPowerByBattery = GPIORead(GPIO_PIN_UPS_POWERED);
}

/* Function to make GPIO state equal to controls[] (see top of file) */
void
ControlStateToGPIO() {
    /* put state on GPIO pins */
    GPIOWrite( GPIO_PIN_PUMP1, CPump1 );
    GPIOWrite( GPIO_PIN_PUMP2, CPump2 );
    GPIOWrite( GPIO_PIN_VALVE, CValve );
    GPIOWrite( GPIO_PIN_EL_HEATER,  CHeater );
}

void
write_log_start() {
    char start_log_text[80];

    log_message(LOG_FILE,"INFO: solard "SOLARDVERSION" now starting up...");
    log_message(LOG_FILE,"Running in "RUNNING_DIR", config file "CONFIG_FILE );
    log_message(LOG_FILE,"PID written to "LOCK_FILE", writing CSV data to "DATA_FILE );
    log_message(LOG_FILE,"writing table data for collectd to "TABLE_FILE );
    log_message(LOG_FILE,"power used persistence file "POWER_FILE );
    sprintf( start_log_text, "powers: heater=%3.1f W, pump1=%3.1f W, pump2=%3.1f W",
    HEATERPPC*(6*60), PUMP1PPC*(6*60), PUMP2PPC*(6*60) );
    log_message(LOG_FILE, start_log_text );
    sprintf( start_log_text, "powers: valve=%3.1f W, self=%3.1f W",
    VALVEPPC*(6*60), SELFPPC*(6*60) );
    log_message(LOG_FILE, start_log_text );
}

/* Function to log currently used config in TABLE_FILE format. The idea is that this file will be made
available to a web app, which will fetch it once in a while to get current working config for solard
without the need for root access (necessary to read /etc/solard.cfg), so relevant data could be shown.
This function should be called less often, e.g. once every 5 minutes or something... */
void
ReWrite_CFG_TABLE_FILE() {
    static char data[280];
    /* Log data like so:
    Time(by log function),mode,wanted_T,use_electric_heater_night,use_electric_heater_day,
	pump1_always_on,use_pump1,use_pump2,day_to_reset_Pcounters,night_boost,abs_max;
	on seperate lines */
    sprintf( data, ",mode,%d\n_,Tboiler_wanted,%d\n_,elh_nt,%d\n_,elh_dt,%d\n"\
    "_,p1_always_on,%d\n_,use_p1,%d\n_,use_p2,%d\n_,Pcounters_rst_day,%d\n"\
	"_,use_night_boost,%d\n_,Tboiler_absMax,%d",
    cfg.mode, cfg.wanted_T, cfg.use_electric_heater_night, cfg.use_electric_heater_day,
	cfg.pump1_always_on, cfg.use_pump1, cfg.use_pump2, cfg.day_to_reset_Pcounters,
	cfg.night_boost, cfg.abs_max);
    log_msg_ovr(CFG_TABLE_FILE, data);
}

/* Function to get current time and put the hour in current_timer_hour */
void
GetCurrentTime() {
    static char buff[80];
    time_t t;
    struct tm *t_struct;
    short adjusted = 0;
    short must_check = 0;
    unsigned short current_day_of_month = 0;
	
	ReWrite_CFG_TABLE_FILE();

    t = time(NULL);
    t_struct = localtime( &t );
    strftime( buff, sizeof buff, "%H", t_struct );

    current_timer_hour = atoi( buff );

    if ((current_timer_hour == 8) && ((ProgramRunCycles % (6*60)) == 0)) must_check = 1;

    /* adjust night tariff start and stop hours at program start and
    every day sometime between 8:00 and 9:00 */
    if (( just_started ) || ( must_check )) {
        strftime( buff, sizeof buff, "%m", t_struct );
        current_month = atoi( buff );
        if ((current_month >= 4)&&(current_month <= 10)) {
            /* April through October - use NE from 23:00 till 6:59 */
            if (NEstart != 23) {
                adjusted = 1;
                NEstart = 23;
                NEstop  = 6;
            }
            now_is_winter = 0;
        }
        else {
            /* November through March - use NE from 22:00 till 5:59 */
            if (NEstart != 22) {
                adjusted = 1;
                NEstart = 22;
                NEstop  = 5;
            }
            now_is_winter = 1;
        }
        if (adjusted) {
            sprintf( buff, "INFO: adjusted night energy hours, start %.2hu:00,"\
            " stop %.2hu:59.", NEstart, NEstop );
            log_message(LOG_FILE, buff);
        }
        /* among other things - manage power used counters; only check one
        time during the day: at 8'something...*/
        if (must_check) {
            strftime( buff, sizeof buff, "%e", t_struct );
            current_day_of_month = atoi( buff );
            if (current_day_of_month == cfg.day_to_reset_Pcounters) {
                /*...if it is the correct day of month - log gathered data and reset counters */
                sprintf( buff, "INFO: Power used last month: nightly: %3.1f Wh, daily: %3.1f Wh;",
                NightlyPowerUsed, (TotalPowerUsed-NightlyPowerUsed) );
                log_message(LOG_FILE, buff);
                sprintf( buff, "INFO: total: %3.1f Wh. Power counters reset.", TotalPowerUsed );
                log_message(LOG_FILE, buff);
                TotalPowerUsed = 0;
                NightlyPowerUsed = 0;
            }
        }
    }
}

void
LogData(short HM) {
    static char data[280];
    /* Log data like so:
        Time(by log function) HOUR, TKOTEL,TSOLAR,TBOILERL,TBOILERH, BOILERTEMPWANTED,BOILERABSMAX,NIGHTBOOST,HM,
    PUMP1,PUMP2,VALVE,EL_HEATER,POWERBYBATTERY, WATTSUSED,WATTSUSEDNIGHTTARIFF */
    sprintf( data, "%2d, %6.3f,%6.3f,%6.3f,%6.3f, %2d,%2d,%d,%2d, %d,%d,%d,%d,%d, %5.3f,%5.3f",\
    current_timer_hour, Tkotel, Tkolektor, TboilerLow, TboilerHigh, cfg.wanted_T, cfg.abs_max, \
    cfg.night_boost, HM, CPump1, CPump2, CValve, CHeater, CPowerByBattery, \
    TotalPowerUsed, NightlyPowerUsed );
    log_message(DATA_FILE, data);

    sprintf( data, ",Temp1,%5.3f\n_,Temp2,%5.3f\n_,Temp3,%5.3f\n_,Temp4,%5.3f\n"\
    "_,Pump1,%d\n_,Pump2,%d\n_,Valve,%d\n_,Heater,%d\n_,PoweredByBattery,%d\n"\
    "_,TempWanted,%d\n_,BoilerTabsMax,%d\n_,ElectricityUsed,%5.3f\n_,ElectricityUsedNT,%5.3f",\
    Tkotel, Tkolektor, TboilerHigh, TboilerLow, CPump1, CPump2,\
    CValve, CHeater, CPowerByBattery, cfg.wanted_T, cfg.abs_max,\
    TotalPowerUsed, NightlyPowerUsed );
    log_msg_ovr(TABLE_FILE, data);

    sprintf( data, "{Tkotel:%5.3f,Tkolektor:%5.3f,TboilerH:%5.3f,TboilerL:%5.3f,"\
    "PumpFurnace:%d,PumpSolar:%d,Valve:%d,Heater:%d,PoweredByBattery:%d,"\
    "TempWanted:%d,BoilerTabsMax:%d,ElectricityUsed:%5.3f,ElectricityUsedNT:%5.3f}",\
    Tkotel, Tkolektor, TboilerHigh, TboilerLow, CPump1, CPump2,\
    CValve, CHeater, CPowerByBattery, cfg.wanted_T, cfg.abs_max,\
    TotalPowerUsed, NightlyPowerUsed );
    log_msg_cln(JSON_FILE, data);
}

/* Return non-zero value on critical condition found based on current data in sensors[] */
short
CriticalTempsFound() {
    if (Tkotel > 68) return 1;
    if (TboilerHigh > 62) return 2;
    return 0;
}

short
BoilerHeatingNeeded() {
    if ( TboilerLow < ((float)cfg.wanted_T - 14) ) return 1;
    if ( TboilerLow > ((float)cfg.wanted_T) ) return 0;
    if ( TboilerHigh < ((float)cfg.wanted_T - 1) ) return 1;
    if ( (TboilerHigh < TboilerHighPrev) && (TboilerHighPrev < (float)cfg.wanted_T) ) return 1;
    return 0;
}

short
SelectIdleMode() {
    short ModeSelected = 0;
    short wantP1on = 0;
    short wantP2on = 0;
    short wantVon = 0;
    short wantHon = 0;

	/* If collector is below 7 C and solar pump has NOT run in the last 15 mins -
		turn pump on to prevent freezing */
	if ((Tkolektor < 7)&&(!CPump2)&&(SCPump2 > (6*15))) wantP2on = 1;
	/* Furnace is above 38 C - at these temps always run the pump */
	if (Tkotel > 38) { wantP1on = 1; }
	else {
		/* below 38 C - check if it is cold to see if we need to run furnace pump:
            if so - run furnace pump at least once every 10 minutes
            we check if it is cold by looking at solar pump idle state - in the cold (-10C)
            it runs at least once per 2 hours; so double that ;) */
		if ((Tkolektor < 33)&&(SCPump2 < (6*60*4))&&(!CPump1)&&(SCPump1 > (6*10))) wantP1on = 1;
	}
    /* Furnace is above 20 C and rising slowly - turn pump on */
    if ((Tkotel > 20)&&(Tkotel > (TkotelPrev+0.12))) wantP1on = 1;
    /* Furnace temp is rising QUICKLY - turn pump on to limit furnace thermal shock */
    if (Tkotel > (TkotelPrev+0.18)) wantP1on = 1;
    /* Do the next checks for boiler heating if boiler is allowed to take heat in */
    if ( (TboilerHigh < (float)cfg.abs_max) ||
         (TboilerLow < (float)(cfg.abs_max - 2)) ) {
        /* Use better heat source: */
        if (Tkolektor > (Tkotel+2)) {
            /* ETCs have heat in excess - build up boiler temp so expensive sources stay idle */
            /* Require selected heat source to be near boiler hot end to avoid loosing heat
            to the enviroment because of the system working */
            if ((Tkolektor > (TboilerLow+12))&&(Tkolektor > (TboilerHigh-2))) wantP2on = 1;
            /* Keep solar pump on while solar fluid is more than 5 C hotter than boiler lower end */
            if ((CPump2) && (Tkolektor > (TboilerLow+4))) wantP2on = 1;
        }
        else {
            /* Furnace has heat in excess - open the valve so boiler can build up
            heat now and probably save on electricity use later on */
            if ((Tkotel > (TboilerHigh+3)) || (Tkotel > (TboilerLow+9)))  {
                wantVon = 1;
                /* And if valve has been open for 90 seconds - turn furnace pump on */
                if (CValve && (SCValve > 8)) wantP1on = 1;
            }
            /* Keep valve open while there is still heat to exploit */
            if ((CValve) && (Tkotel > (TboilerLow+4))) wantVon = 1;
        }
    }
    /* Try to heat the house by taking heat from boiler but leave at least 2 C extra on
    top of the wanted temp - first open the valve, then turn furnace pump on */
    if ( (cfg.mode==2) && /* 2=AUTO+HEAT HOUSE BY SOLAR; */
    (TboilerHigh > ((float)cfg.wanted_T + 2)) && (TboilerLow > (Tkotel + 8)) ) {
        wantVon = 1;
        /* And if valve has been open for 1 minute - turn furnace pump on */
        if (CValve && (SCValve > 6)) wantP1on = 1;
    }
    /* Run solar pump once every day at the predefined hour for current month (see array definition)
    if it stayed off the past 4 hours*/
    if ( (current_timer_hour == pump_start_hour_for[current_month]) && 
         (!CPump2) && (SCPump2 > (6*60*4)) ) wantP2on = 1;
    if (cfg.pump1_always_on) {
        wantP1on = 1;
    }
    else {
        /* Turn furnace pump on every 4 days */
        if ( (!CPump1) && (SCPump1 > (6*60*24*4)) ) wantP1on = 1;
    }
    /* Prevent ETC from boiling its work fluid away in case all heat targets have been reached
        and yet there is no use because for example the users are away on vacation */
    if (Tkolektor > 68) {
        wantVon = 1;
        /* And if valve has been open for ~1.5 minutes - turn furnace pump on */
        if (CValve && (SCValve > 8)) wantP1on = 1;
        /* And if valve has been open for 2 minutes - turn solar pump on */
        if (CValve && (SCValve > 11)) wantP2on = 1;
    }
    /* Two energy saving functions follow (if activated): */
    /* 1) During night tariff hours, try to keep boiler lower end near wanted temp */
    if ( (current_timer_hour <= NEstop) || (current_timer_hour >= NEstart) ) {
        if ( (!CPump2) && (TboilerLow < ((float)cfg.wanted_T - 1.1)) ) { wantHon = 1; }
    }
    /* 2) In the last 2 hours of night energy tariff heat up boiler until the lower sensor
    reads 12 C on top of desired temp, clamped at cfg.abs_max, so that less day energy gets used */
    if ( (cfg.night_boost) && (current_timer_hour >= (NEstop-1)) && (current_timer_hour <= NEstop) ) {
        if (TboilerLow < nightEnergyTemp) { wantHon = 1; }
    }

    if ( wantP1on ) ModeSelected |= 1;
    if ( wantP2on ) ModeSelected |= 2;
    if ( wantVon )  ModeSelected |= 4;
    if ( wantHon )  ModeSelected |= 8;
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
    if ((Tkolektor > (TboilerLow + 10))&&(Tkolektor > Tkotel)) {
        /* To enable solar heating, ETC temp must be at least 10 C higher than boiler cold end */
        wantP2on = 1;
    }
    else {
        /* Not enough heat in the solar collector; check other sources of heat */
        if (Tkotel > (TboilerLow + 9)) {
            /* The furnace is hot enough - use it */
            wantVon = 1;
            /* And if valve has been open for 2 minutes - turn furnace pump on */
            if (CValve &&(SCValve > 13)) wantP1on = 1;
        }
        else {
            /* All is cold - use electric heater if possible */
            /* Only turn heater on if valve is fully closed, because it runs with at least one pump
               and make sure ETC pump is NOT running...*/
            if ((!CValve && (SCValve > 15))&&(!CPump2)) wantHon = 1;
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
void TurnPump1On()   { if (cfg.use_pump1 && (!CPump1) && (SCPump1 > 2)) { CPump1 = 1; SCPump1 = 0; } }
void TurnPump2Off()  { if (CPump2 && (SCPump2 > 5)) { CPump2  = 0; SCPump2 = 0; } }
void TurnPump2On()   { if (cfg.use_pump2 && (!CPump2) && (SCPump2 > 2)) { CPump2  = 1; SCPump2 = 0; } }
void TurnValveOff()  { if (CValve && (SCValve > 17)) { CValve  = 0; SCValve = 0; } }
void TurnValveOn()   { if (!CValve && (SCValve > 5)) { CValve  = 1; SCValve = 0; } }
void TurnHeaterOff() { if (CHeater && (SCHeater > 17)) { CHeater = 0; SCHeater = 0; } }
void TurnHeaterOn()  { if ((!CHeater) && (SCHeater > 29)) { CHeater = 1; SCHeater = 0; } }

void
RequestElectricHeat() {
    /* Do the check with config to see if its OK to use electric heater,
    for example: if its on "night tariff" - switch it on */
    /* Determine current time: */
    if ( (current_timer_hour <= NEstop) || (current_timer_hour >= NEstart) ) {
            /* NIGHT TARIFF TIME */
            /* If heater use is allowed by config - turn it on */
            if (cfg.use_electric_heater_night) TurnHeaterOn();
    }
    else {
            /* DAY TIME */
            /* If heater use is allowed by config - turn it on */
            if (cfg.use_electric_heater_day) TurnHeaterOn();
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
        TotalPowerUsed += PUMP1PPC;
        if ( (current_timer_hour <= NEstop) || (current_timer_hour >= NEstart) ) { NightlyPowerUsed += PUMP1PPC; }
    }
    if ( CPump2 ) {
        TotalPowerUsed += PUMP2PPC;
        if ( (current_timer_hour <= NEstop) || (current_timer_hour >= NEstart) ) { NightlyPowerUsed += PUMP2PPC; }
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
AdjustHeatingModeForBatteryPower(unsigned short HM) {
    /* Check for power source switch */
    if ( CPowerByBattery != CPowerByBatteryPrev ) {
        /* If we just switched to battery.. */
        if ( CPowerByBattery ) {
            log_message(LOG_FILE,"WARNING: Switch to BATTERY POWER detected.");
        }
        else {
            log_message(LOG_FILE,"INFO: Powered by GRID now.");
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
    unsigned short iter = 30;
    unsigned short iter_P = 0;
    unsigned short AlarmRaised = 0;
    unsigned short HeatingMode = 0;
    struct timeval tvalBefore, tvalAfter;

    SetDefaultCfg();

    /* before main work starts - try to open the log files to write a new line
    ...and SCREAM if there is trouble! */
    if (log_message(LOG_FILE,"***\n")) {
        printf(" Cannot open the mandatory "LOG_FILE" file needed for operation!\n");
        exit(3);
    }
    if (log_message(DATA_FILE,"***\n")) {
        printf(" Cannot open the mandatory "DATA_FILE" file needed for operation!\n");
        exit(3);
    }
    if (log_message(TABLE_FILE,"***\n")) {
        printf(" Cannot open the mandatory "TABLE_FILE" file needed for operation!\n");
        exit(3);
    }
    if (log_message(JSON_FILE,"***\n")) {
        printf(" Cannot open the mandatory "JSON_FILE" file needed for operation!\n");
        exit(3);
    }
    if (log_message(CFG_TABLE_FILE,"***\n")) {
        printf(" Cannot open the mandatory "CFG_TABLE_FILE" file needed for operation!\n");
        exit(4);
    }

    daemonize();

    /* Enable GPIO pins */
    if ( ! EnableGPIOpins() ) {
        log_message(LOG_FILE,"ALARM: Cannot enable GPIO! Aborting run.");
        return(1);
    }

    /* Set GPIO directions */
    if ( ! SetGPIODirection() ) {
        log_message(LOG_FILE,"ALARM: Cannot set GPIO direction! Aborting run.");
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
            log_message(LOG_FILE,"WARNING: error getting tvalBefore...");
        }
        /* get the current hour every 5 minutes for electric heater schedule */
        if ( iter == 30 ) {
            iter = 0;
            GetCurrentTime();
            /* and increase counter controlling writing out persistent power use data */
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
        switch (cfg.mode) {
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
                    log_message(LOG_FILE,"ALARM: Activating emergency cooling!");
                    AlarmRaised = 1;
                }
            }
            else {
                if ( AlarmRaised ) {
                    log_message(LOG_FILE,"INFO: Critical condition resolved. Running normally.");
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
            log_message(LOG_FILE,"WARNING: error getting tvalAfter...");
            sleep( 7 );
        }
        else {
            /* use hardcoded sleep() if time is skewed (for eg. daylight saving, ntp adjustments, etc.) */
            if ((tvalAfter.tv_sec - tvalBefore.tv_sec) > 12) {
                sleep( 7 );
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
        log_message(LOG_FILE,"Cannot disable GPIO on exit!");
        return(4);
    }

    return(0);
}

/* EOF */
