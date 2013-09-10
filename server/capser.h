int writeport(int fd, char *chars);
int readport(int fd, char *result);
int readport_blocking(int fd, char *result, char *end);
int getbaud(int fd);
//int initport();
