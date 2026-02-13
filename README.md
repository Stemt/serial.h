# serial.h

A single header serial library for Windows and Linux.

## Simple Example

```c
#define SERIAL_IMPLEMENTATION
#include "../serial.h"

#include <stdio.h>
#include <string.h>

int main(int argc, char** argv){

  SerialPort port = {0};

  const char* port_path = "/dev/ttyUSB0";

  // opens port with default settings, equivalent to
  // ```
  //   serial_open(&port, port_path,
  //      .baud_rate      = 9600,
  //      .char_size      = 8,
  //      .flow_control   = SERIAL_FLOWCTRL_NONE,
  //      .parity         = SERIAL_PARITY_NONE
  //      .stop_bits      = SERIAL_STOPBITS_NONE
  //      .timeout_ms     = SERIAL_TIMEOUT_INFINITE
  //   )
  // ```
  if(serial_open(&port, port_path) == false){
    fprintf(stderr, "Failed to open port %s: %s\n", port_path, serial_strerror(serial_error(&port)));
    return 1;
  }

  const char* msg = "Hello Device!";
  if(serial_write(&port, msg, strlen(msg)) < 0){
    fprintf(stderr, "Failed to write to port: %s\n", serial_strerror(serial_error(&port)));
    return 1;
  }

  char buf[1024] = {0};
  int n_read = serial_read(&port, buf, sizeof(buf)-1);
  if(n_read < 0){
    fprintf(stderr, "Failed to write to port: %s\n", serial_strerror(serial_error(&port)));
    return 1;
  }else if(n_read == 0){
    fprintf(stderr, "Port read timed out");
    return 1;
  }else{
    fprintf(stderr, "Received: '%s'\n", buf);
  }

  return 0;
}
```

