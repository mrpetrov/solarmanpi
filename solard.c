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

    # mode: 0=ALL OFF; 1=AUTO; 2=AUTO+HEAT HOUSE BY SOLAR;
    #  3=MANAUL PUMP1+HEATER; 4=MANUAL PUMP1 ONLY; 5=MANUAL PUMP2 ONLY;
    mode=1

    # wanted_T: the desired temperature of water in tank
    wanted_T=50

    # these define allowed hours to use electric heat
    use_electric_start_hour=4
    use_electric_stop_hour=5

    # this tells to default pump1 to ON in idles
    keep_pump1_on=1
*/

#define SOLARDVERSION    "2.2 2015-04-15"

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
#define DATA_FILE       "/run/shm/solard_data.csv"
#define TABLE_FILE      "/run/shm/solard_current"
#define CONFIG_FILE     "/etc/solard.cfg"

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

/* Number of all sensors to be used by the system */
#define TOTALSENSORS         4

/* Define the DS18B20 sensors paths to read temps from */
#define SENSOR1 "/sys/bus/w1/devices/28-041464764cff/w1_slave"
#define SENSOR2 "/sys/bus/w1/devices/28-041464403bff/w1_slave"
#define SENSOR3 "/sys/bus/w1/devices/28-04146448f3ff/w1_slave"
#define SENSOR4 "/sys/bus/w1/devices/28-0214608d40ff/w1_slave"

char *sensor_paths[] = { SENSOR1, SENSOR2, SENSOR3, SENSOR4 };

/* var to keep track of read errors, so if a threshold is reached - the
program can safely shut down everything, send notification and bail out */
unsigned int sensor_read_errors = (11*TOTALSENSORS);

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
short controls[11] = { -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

/* and control name mappings */
#define   CPump1                controls[1]
#define   CPump2                controls[2]
#define   CValve                controls[3]
#define   CHeater               controls[4]
#define   CPump1Prev            controls[5]
#define   CPump2Prev            controls[6]
#define   CValvePrev            controls[7]
#define   CHeaterPrev           controls[8]
#define   CPowerByBattery       controls[9]
#define   CPowerByBatteryPrev   controls[10]

/* controls state cycles - zeroed on change to state */
long ctrlstatecycles[5] = { -1, 0, 0, 0, 0 };

#define   SCPump1               ctrlstatecycles[1]
#define   SCPump2               ctrlstatecycles[2]
#define   SCValve               ctrlstatecycles[3]
#define   SCHeater              ctrlstatecycles[4]

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
}
structsolard_cfg;

struct structsolard_cfg solard_cfg;

short need_to_read_cfg = 0;

unsigned short current_timer_hour = 0;

short just_started = 0;

/* FORWARD DECLARATIONS so functions can be used in preceeding ones */

int
DisableGPIOpins();

/* end of forward-declared functions */

void
SetDefaultCfg() {
    strcpy( solard_cfg.mode_str, "1");
    solard_cfg.mode = 1;
    strcpy( solard_cfg.wanted_T_str, "50");
    solard_cfg.wanted_T = 50;
    strcpy( solard_cfg.use_electric_start_hour_str, "4");
    solard_cfg.use_electric_start_hour = 4;
    strcpy( solard_cfg.use_electric_stop_hour_str, "5");
    solard_cfg.use_electric_stop_hour = 5;
    strcpy( solard_cfg.keep_pump1_on_str, "1");
    solard_cfg.keep_pump1_on = 1;
}

static void
log_message(char *filename, char *message) {
    FILE *logfile;
    char msg[100];
    char buff[70];
    time_t t;
    struct tm *t_struct;

    t = time(NULL);
    t_struct = localtime( &t );
    strftime( buff, sizeof buff, "%F %T", t_struct );
    sprintf( msg, "%s%s", buff, message );
    logfile = fopen( filename, "a" );
    if ( ! logfile ) return;
    fprintf( logfile, "%s\n", msg );
    fclose( logfile );
}

/* this version of the logging function destroys the opened file contents */
static void
log_msg_ovr(char *filename, char *message) {
    FILE *logfile;
    char msg[100];
    char buff[70];
    time_t t;
    struct tm *t_struct;

    t = time(NULL);
    t_struct = localtime( &t );
    strftime( buff, sizeof buff, "%F %T", t_struct );
    sprintf( msg, "%s%s", buff, message );
    logfile = fopen( filename, "w" );
    if ( ! logfile ) return;
    fprintf( logfile, "%s\n", msg );
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

static void
parse_config()
{
    int i = 0;
    char *s, buff[256];
    FILE *fp = fopen(CONFIG_FILE, "r");
    if (fp == NULL) {
        log_message(LOG_FILE," Failed to open "CONFIG_FILE" file for reading!");
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
    if ( i ) solard_cfg.use_electric_start_hour = i;
    strcpy( buff, solard_cfg.use_electric_stop_hour_str );
    i = atoi( buff );
    if ( i ) solard_cfg.use_electric_stop_hour = i;
    strcpy( buff, solard_cfg.keep_pump1_on_str );
    i = atoi( buff );
    solard_cfg.keep_pump1_on = i;

    /* Prepare log message and write it to log file */
    if (fp == NULL) {
        sprintf( buff, " Using values: M=%d, Twanted=%d, ELH start=%d, stop=%d, keepP1on=%d",\
        solard_cfg.mode, solard_cfg.wanted_T, solard_cfg.use_electric_start_hour, \
        solard_cfg.use_electric_stop_hour, solard_cfg.keep_pump1_on );
        } else {
        sprintf( buff, " Read CFG file: M=%d, Twanted=%d, ELH start=%d, stop=%d, keepP1on=%d",\
        solard_cfg.mode, solard_cfg.wanted_T, solard_cfg.use_electric_start_hour, \
        solard_cfg.use_electric_stop_hour, solard_cfg.keep_pump1_on );
    }
    log_message(LOG_FILE, buff);
}

static int
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

static int
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

static int
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

static int
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

static int
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

static float
sensorRead(const char* sensor)
{
    char path[VALUE_MAX];
    char value_str[80];
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
        return(temp);
    }

    /* throw the first line away */
    strncpy(value_str, " ", 80);

    /* read the second line into value_str */
    if (-1 == read(fd, value_str, 35)) {
        log_message(LOG_FILE," Error reading row 2 from sensor file. Continuing.");
        return(temp);
    }

    /* close the file - we are done with it */
    close(fd);

    /* transform sensor data to float */
    if((result = strchr((char *)&value_str, '=')) != NULL) {
        ++result; // increment result to avoid the '='
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
        log_message(LOG_FILE, " Signal SIGUSR1 caught. Will re-read config file soon.");
        need_to_read_cfg = 1;
        break;
        case SIGUSR2:
        log_message(LOG_FILE, " Signal SIGUSR2 caught. Not implemented. Continuing.");
        break;
        case SIGHUP:
        log_message(LOG_FILE, " Signal SIGHUP caught. Not implemented. Continuing.");
        break;
        case SIGTERM:
        log_message(LOG_FILE, " Terminate signal caught. Stopping.");
        if ( ! DisableGPIOpins() ) { log_message(LOG_FILE, " Cannot disable GPIO! Quitting."); exit(4); }
        log_message(LOG_FILE," Exiting normally. Bye, bye!");
        exit(0);
        break;
    }
}

void
daemonize()
{
    int i,lfp;
    char str[10];

    if(getppid()==1) return; /* already a daemon */
    i=fork();
    if (i<0) exit(1); /* fork error */
    if (i>0) exit(0); /* parent exits */
    /* child (daemon) continues */
    setsid(); /* obtain a new process group */
    for (i=getdtablesize();i>=0;--i) close(i); /* close all descriptors */
    i=open("/dev/null",O_RDWR); dup(i); dup(i); /* handle standart I/O */
    umask(022); /* set newly created file permissions */
    chdir(RUNNING_DIR); /* change running directory */
    lfp=open(LOCK_FILE,O_RDWR|O_CREAT,0644);
    if (lfp<0) exit(1); /* can not open */
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
int
EnableGPIOpins()
{
    if (-1 == GPIOExport(PUMP1)) return 0;
    if (-1 == GPIOExport(PUMP2)) return 0;
    if (-1 == GPIOExport(VALVE)) return 0;
    if (-1 == GPIOExport(HEAT))  return 0;
    if (-1 == GPIOExport(POWER)) return 0;
    return -1;
}

int
SetGPIODirection()
{
    if (-1 == GPIODirection(PUMP1, OUT)) return 0;
    if (-1 == GPIODirection(PUMP2, OUT)) return 0;
    if (-1 == GPIODirection(VALVE, OUT)) return 0;
    if (-1 == GPIODirection(HEAT, OUT))  return 0;
    if (-1 == GPIODirection(POWER, IN))  return 0;
    return -1;
}

int
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
            if (sensor_read_errors) sensor_read_errors--;
            if (just_started) { sensors[i+5] = new_val; sensors[i+1] = new_val; }
            if ((new_val < (sensors[i+5]-4))) {
                sprintf( msg, " WARNING: Correcting LOW %3.3f for sensor %d with %3.3f.",\
                new_val, i+1, sensors[i+5]-4 );
                log_message(LOG_FILE, msg);
                new_val = sensors[i+5]-4;
            }
            if ((new_val > (sensors[i+5]+4))) {
                sprintf( msg, " WARNING: Correcting HIGH %3.3f for sensor %d with %3.3f.",\
                new_val, i+1, sensors[i+5]+4 );
                log_message(LOG_FILE, msg);
                new_val = sensors[i+5]+4;
            }
            sensors[i+5] = sensors[i+1];
            sensors[i+1] = new_val;
        }
        else {
            sensor_read_errors++;
            sprintf( msg, " WARNING: ReadSensors() errors++ at sensor %d.", i+1 );
            log_message(LOG_FILE, msg);
        }
    }
    if (sensor_read_errors>(12*TOTALSENSORS)){
        /* log the errors, clean up and bail out */
        if ( ! DisableGPIOpins() ) {
            log_message(LOG_FILE, " ALARM: Too many sensor errors! GPIO disable failed.");
            exit(5);
        }
        log_message(LOG_FILE, " ALARM: Too many sensor read errors! Stopping.");
        exit(10);
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
int
CriticalTempsFound() {
    if (Tkotel >= 77) return 1;
    if (Tkolektor >= 88) return 2;
    if (TboilerHigh >= 70) return 3;
    if (Tkolektor <= 4) return -1;
    return 0;
}

/* Function to make GPIO state equal to controls[] (see top of file) */
void
ControlStateToGPIO() {
    /* put state on GPIO pins */
    GPIOWrite( PUMP1, CPump1 );
    /* Do not physically change PUMP2 state - currently it is absent, so just log it
        GPIOWrite( PUMP2, CPump2 );
    */
    GPIOWrite( VALVE, CValve );
    GPIOWrite( HEAT,  CHeater );
}

/* Function to get current time and put the hour in current_timer_hour */
void
GetCurrentTime() {
    static char buff[5];
    time_t t;
    struct tm *t_struct;

    t = time(NULL);
    t_struct = localtime( &t );
    strftime( buff, sizeof buff, "%H", t_struct );

    current_timer_hour = atoi( buff );
}

/* Function to open the valve, and start all pumps */
void
ActivateEmergencyHeatTransfer() {
    /* first set controls[]' states */
    CPump1 = 1;
    CPump2 = 1;
    CValve = 1;
    CHeater = 0;
    /* and then put state on GPIO pins */
    ControlStateToGPIO();
}

int
BoilerHeatingNeeded() {
    /* Calculate boiler temp as intermediary between the boiler hot and cold ends */
    /* if ( ((TboilerLow + TboilerHigh) / 2) <= ((float) solard_cfg.wanted_T) ) return 1; */
    /* Check just the higher temp sensor to determine if boiler needs heating */
    if ( TboilerHigh < (float) solard_cfg.wanted_T ) return 1;
    else return 0;
}

static void
LogData(int HM) {
    char msg[100];
    /* Log data like so:
        Time(by log function), TKOTEL,TSOLAR,TBOILERL,TBOILERH,
    BOILERTEMPWANTED,HM, PUMP1,PUMP2,VALVE,HEAT,POWERBYBATTERY */
    sprintf( msg, ", %3.3f,%3.3f,%3.3f,%3.3f, %d,%d, %d,%d,%d,%d,%d",\
    Tkotel, Tkolektor, TboilerLow, TboilerHigh, solard_cfg.wanted_T, HM,\
    CPump1, CPump2, CValve, CHeater, CPowerByBattery );
    log_message(DATA_FILE, msg);

    sprintf( msg, ",Temp1,%3.3f", Tkotel );
    log_msg_ovr(TABLE_FILE, msg);
    sprintf( msg, ",Temp2,%3.3f", Tkolektor );
    log_message(TABLE_FILE, msg);
    sprintf( msg, ",Temp3,%3.3f", TboilerHigh );
    log_message(TABLE_FILE, msg);
    sprintf( msg, ",Temp4,%3.3f", TboilerLow );
    log_message(TABLE_FILE, msg);
    sprintf( msg, ",Pump1,%d", CPump1 );
    log_message(TABLE_FILE, msg);
    sprintf( msg, ",Pump2,%d", CPump2 );
    log_message(TABLE_FILE, msg);
    sprintf( msg, ",Valve,%d", CValve );
    log_message(TABLE_FILE, msg);
    sprintf( msg, ",Heater,%d", CHeater );
    log_message(TABLE_FILE, msg);
    sprintf( msg, ",PoweredByBattery,%d", CPowerByBattery );
    log_message(TABLE_FILE, msg);
    sprintf( msg, ",TempWanted,%d", solard_cfg.wanted_T );
    log_message(TABLE_FILE, msg);
}

short
SelectIdleMode() {
    short ModeSelected = 0;
	short wantP1on = 0;
	short wantP2on = 0;
	short wantVon = 0;
    /* If furnace is cold - turn pump on to keep it from freezing */
    if (Tkotel < 8.9)
    wantP1on = 1;
    /* If solar is cold - turn pump on to keep it from freezing */
    if (Tkolektor < 8.9)
    wantP2on = 1;
    /* Furnace is above 44 and rising - turn pump on */
    if ((Tkotel > 43.8)&&(Tkotel > TkotelPrev))
    wantP1on = 1;
    /* Furnace is above 38 and rising slowly - turn pump on */
    if ((Tkotel > 37.8)&&(Tkotel > (TkotelPrev+0.06)))
    wantP1on = 1;
    /* Furnace is above 24 and rising QUICKLY - turn pump on to limit furnace thermal shock */
    if ((Tkotel > 23.8)&&(Tkotel > (TkotelPrev+0.18)))
    wantP1on = 1;
    /* If solar is 12 C hotter than furnace and we want to heat the house
    - turn both pumps on and open the valve */
	if ( (solard_cfg.mode=2) && /* 2=AUTO+HEAT HOUSE BY SOLAR; */
    ((Tkolektor > (Tkotel+11.9))&&(Tkolektor > TkolektorPrev)) ) {
        wantP1on = 1;
        wantP2on = 1;
        wantVon = 1;
    }
	/* If furnace pump has been off for 7 days - turn it on for a while, to circulate fluid,
    keep it in shape */
    if ( (!CPump1) && (SCPump1 > 7*24*60*60) )
    wantP1on = 1;
	/* If solar pump has been off for 20 minutes - turn it on for a while, to circulate fluid */
    if ( (!CPump2) && (SCPump2 > 6*20) )
    wantP2on = 1;
    if (solard_cfg.keep_pump1_on) wantP1on = 1;

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

    /* Do in main select routine what the idle routine would do + extra bits: */
    ModeSelected = SelectIdleMode();

    if ((Tkolektor > (TboilerHigh + 4.9))&&(Tkolektor > Tkotel)) {
        /* To enable solar heating, solar out temp must be at least 5 C higher than the boiler */
        wantP2on = 1;
    }
    else {
        /* Not enough heat in the solar collector; check other sources of heat */
        if (Tkotel > (TboilerHigh + 1.5)) {
            /* The furnace is hot enough - use it */
            wantP1on = 1;
            wantVon = 1;
            } else {
            /* All is cold - use electric heater if possible */
            wantHon = 1;
        }
    }

	if ( wantP1on ) ModeSelected |= 1;
	if ( wantP2on ) ModeSelected |= 2;
	if ( wantVon )  ModeSelected |= 4;
	if ( wantHon )  ModeSelected |= 8;
    return ModeSelected;
}

void TurnPump1Off()  { if ((CPump1 && !CValve) && (SCPump1 > 5)) { CPump1  = 0; SCPump1 = 0; } }
void TurnPump1On()   { if (!CPump1) { CPump1  = 1; SCPump1 = 0; } }
void TurnPump2Off()  { if (CPump2 && (SCPump2 > 5)) { CPump2  = 0; SCPump2 = 0; } }
void TurnPump2On()   { if (!CPump2) { CPump2  = 1; SCPump2 = 0; } }
void TurnValveOff()  { if (CValve && (SCValve > 15)) { CValve  = 0; SCValve = 0; } }
void TurnValveOn()   { if (!CValve) { CValve  = 1; SCValve = 0; } }
void TurnHeaterOff() { if (CHeater) { CHeater = 0; SCHeater = 0; } }
void TurnHeaterOn()  { if ((!CHeater) && (SCHeater > 5)) { CHeater = 1; SCHeater = 0; } }

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
        } else {
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
    /* Check for power failure */
    if ( CPowerByBattery != CPowerByBatteryPrev ) {
        /* If we just switched to battery.. */
        if ( CPowerByBattery ) {
            /* turn everything OFF */
            HM = 0;
            log_message(LOG_FILE," WARNING: Switch to BATTERY POWER detected.");
        }
        else {
            log_message(LOG_FILE," INFO: Powered by GRID now.");
        }
    }
}

int
main(int argc, char *argv[])
{
    /* set iter to its max value - makes sure we get a clock reading upon start */
    short iter = 30;
    short AlarmRaised = 0;
    short HeatingMode = 0;
    struct timeval tvalBefore, tvalAfter;

    SetDefaultCfg();

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

    log_message(LOG_FILE," solard "SOLARDVERSION" now starting up...");
    log_message(LOG_FILE," Running in "RUNNING_DIR", config file "CONFIG_FILE );
    log_message(LOG_FILE," PID written to "LOCK_FILE", writing CSV data to "DATA_FILE );
    log_message(LOG_FILE," writing table data for collectd to "TABLE_FILE );

    parse_config();

    just_started = 1;

    do {
        /* Do all the important stuff... */
        gettimeofday( &tvalBefore, NULL );
        /* get the current hour every 5 minutes for electric heater schedule */
        if ( iter == 30 ) {
            iter = 0;
            GetCurrentTime();
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
                ActivateEmergencyHeatTransfer();
                if ( !AlarmRaised ) {
                    log_message(LOG_FILE," ALARM: Activating emergency cooling!");
                    AlarmRaised = 1;
                }
            }
            else {
                if ( AlarmRaised ) {
                    log_message(LOG_FILE," Critical condition resolved. Running normally.");
                    AlarmRaised = 0;
                }
                if (BoilerHeatingNeeded()) {
                    HeatingMode = SelectHeatingMode();
                    } else {
                    /* No heating needed - decide how to idle */
                    HeatingMode = SelectIdleMode();
                }
                /*AdjustHeatingModeForBatteryPower(HeatingMode);*/
            }
            break;
            case 3: /* 3=MANAUL PUMP1+HEATER - furnace pump and heater power ON */
            HeatingMode = 17;
            /* But still manage pumps in case of power failure */
            /*AdjustHeatingModeForBatteryPower(HeatingMode);*/
            break;
            case 4: /* 4=MANUAL PUMP1 ONLY - only furnace pump ON */
            HeatingMode = 1;
            /* But still manage pumps in case of power failure */
            /*AdjustHeatingModeForBatteryPower(HeatingMode);*/
            break;
            case 5: /* 5=MANUAL PUMP2 ONLY - only solar pump ON */
            HeatingMode = 2;
            /* But still manage pumps in case of power failure */
            /*AdjustHeatingModeForBatteryPower(HeatingMode);*/
            break;
        }
        ActivateHeatingMode(HeatingMode);
        LogData(HeatingMode);
        if ( need_to_read_cfg ) {
            need_to_read_cfg = 0;
            parse_config();
        }
        if ( just_started ) { just_started = 0; }
        gettimeofday( &tvalAfter, NULL );
        /* ..and sleep for rest of the 10 seconds wait period */
        usleep(10000000 - (((tvalAfter.tv_sec - tvalBefore.tv_sec)*1000000L \
        + tvalAfter.tv_usec) - tvalBefore.tv_usec));
    } while (1);

    /* Disable GPIO pins */
    if ( ! DisableGPIOpins() ) {
        log_message(LOG_FILE," Cannot disable GPIO! Aborting run.");
        return(4);
    }

    return(0);
}

/* EOF */
