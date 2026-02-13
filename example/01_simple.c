#define SERIAL_IMPLEMENTATION
#include "../include/serial.h"

#define NOB_IMPLEMENTATION
#include "../nob.h"

int main(int argc, char** argv){
  const char* port_path = nob_shift_args(&argc, &argv);
#if 0 
  const char* baud_str = nob_shift_args(&argc, &argv);

  char* endptr = NULL;
  int baud = strtol(baud_str, &endptr, 10);
  if(endptr == baud_str){
    nob_log(NOB_ERROR, "Invalid baud: '%s'", baud_str);
    return 1;
  }
#endif

  SerialPort port = {0};
  if(!serial_open(&port, port_path,
      .baud_rate = 9600,
      .char_size = 8
  )){
    nob_log(NOB_ERROR, "Failed to open port: '%s'", port_path);
    return 1;
  }
}
