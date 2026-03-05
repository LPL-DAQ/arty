
#ifndef APP_SNTP_IMP_H
#define APP_SNTP_IMP_H


bool sync_time();
int sntp_init(void);
timespec get_system_time();

#endif // APP_SNTP_IMP_H
