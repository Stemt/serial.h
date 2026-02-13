/*
MIT License

Copyright (c) 2019 Patrick Rollings

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef SERIAL_HPP
#define SERIAL_HPP

#if __linux__
#define SERIAL_OS_LINUX 1
#elif __APPLE__
#define SERIAL_OS_APPLE 1
#endif

#ifdef _WIN32
#define SERIAL_OS_WINDOWS 1
#endif

#include <stdbool.h>

#if SERIAL_OS_WINDOWS
#include <windows.h>
#include <stdio.h>
#include <stdbool.h>

#elif SERIAL_OS_LINUX
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <linux/serial.h>
#include <sys/select.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#endif

typedef enum{
  SERIAL_FLOWCTRL_NONE = 0,
  SERIAL_FLOWCTRL_SOFTWARE,
  SERIAL_FLOWCTRL_HARDWARE,
} SerialFlowControl;

typedef enum{
  SERIAL_PARITY_NONE = 0,
  SERIAL_PARITY_ODD,
  SERIAL_PARITY_EVEN,
} SerialParity;

typedef enum{
  SERIAL_STOPBITS_ONE = 0,
  SERIAL_STOPBITS_ONE_POINT_FIVE,
  SERIAL_STOPBITS_TWO,
} SerialStopBits;

#define SERIAL_TIMEOUT_INFINITE 0
#define SERIAL_TIMEOUT_NONE -1

typedef struct{
  int baud_rate;
  int char_size;
  SerialFlowControl flow_control;
  SerialParity parity;
  SerialStopBits stop_bits;
  int timeout_ms;
} SerialSettings;

typedef enum{
  SERIAL_ERR_NONE = 0,
  SERIAL_ERR_DISCONNECT,
  SERIAL_ERR_INVALID_DEVICE,
  SERIAL_ERR_INVALID_BAUD,
  SERIAL_ERR_INVALID_CHAR_SIZE,
  SERIAL_ERR_INVALID_STOP_BITS,
  SERIAL_ERR_INVALID_TIMEOUT,
  SERIAL_ERR_TIMEOUT,
  SERIAL_ERR_PLATFORM,
} SerialError;

#if SERIAL_OS_WINDOWS
typedef HANDLE SerialNativeHandle;
#elif SERIAL_OS_LINUX
typedef int SerialNativeHandle;
#endif

typedef struct{
  const char* path;
  SerialNativeHandle handle;
  SerialSettings settings;
  SerialError err;
} SerialPort;

#define serial_open(port, device, ...)\
  serial_open_with_settings(port, device, (SerialSettings){ __VA_ARGS__ })

bool serial_open_with_settings(SerialPort* self, const char* device, SerialSettings settings);
SerialSettings serial_get_settings(SerialPort* self);
bool serial_reconfigure(SerialPort* self, SerialSettings settings);

int serial_read(SerialPort* self, char* buf, int length);
int serial_write(SerialPort* self, const char* buf, int length);

#define SERIAL_IMPLEMENTATION

#ifdef SERIAL_IMPLEMENTATION
#if SERIAL_OS_WINDOWS
int serial_get_all_device_subkeys(char*** subkeys) {
  TCHAR* key_path = (TCHAR*)"SYSTEM\\CurrentControlSet\\Services\\FTDIBUS\\Enum";
  HKEY key;
  auto r = RegOpenKeyEx(HKEY_LOCAL_MACHINE, key_path, 0, KEY_READ, &key);
  if (r)
  {
    return 0; // or -1?
  }

  DWORD type;
  DWORD count;
  DWORD count_size = sizeof(DWORD);
  r = RegQueryValueEx(key, "Count", NULL, &type, (LPBYTE)&count, &count_size);
  *subkeys = calloc(count, sizeof(char*));
  DWORD port_info_size = 140;
  TCHAR port_info[port_info_size];
  for (int iii = 0; iii < count; iii++)
  {
    char num[10];
    sprintf(num, "%d", iii);
    RegQueryValueEx(
      key, num, NULL, &type, (LPBYTE)&port_info, &port_info_size
    );

    int port_info_len = strlen(port_info) - 4;
    char* subkey = calloc(port_info_len + 1, 1);
    strcpy(subkey, &port_info[4]);
    subkey[port_info_len] = 'A';
    subkey[strcspn(subkey, "&")] = '+';
    subkey[strcspn(subkey, "\\")] = '+';
    subkeys[iii] = &subkey;
  }
  RegCloseKey(key);
  return count;
}

HKEY serial_open_device_params(char* port_name) {
  char** device_subkeys;
  auto device_count = get_all_device_subkeys(&device_subkeys);
  HKEY key;
  TCHAR* keypath = (TCHAR*)"SYSTEM\\CurrentControlSet\\Enum\\FTDIBUS";
  long r = RegOpenKeyEx(HKEY_LOCAL_MACHINE, keypath, 0, KEY_READ, &key);
  if (r) {
    // no drives
  }

  int index = 0;
  HKEY read_only_key = 0, param_key = 0;
  for (int iii = 0; iii < device_count; iii++) {
    char* subkey = device_subkeys[iii];
    TCHAR param_path[100];
    snprintf(param_path, 100, "%s\\0000\\Device Parameters", subkey);
    r = RegOpenKeyEx(key, param_path, 0, KEY_READ, &read_only_key);
    if (r) {
      continue;
    }
    DWORD reg_port_name_size = 10;
    TCHAR reg_port_name[reg_port_name_size];
    DWORD type;
    r = RegQueryValueEx(
      read_only_key, "PortName", NULL, &type, (LPBYTE)&reg_port_name, &reg_port_name_size
    );
    bool names_match = strcmp(port_name, reg_port_name) == 0;
    if (!r && names_match) {
      r = RegOpenKeyEx(read_only_key, NULL, 0, KEY_READ | KEY_SET_VALUE, &param_key);
      if (r) {
        // admin priv error
      }
    }
    RegCloseKey(read_only_key);
    if (param_key) {
      break;
    }
  }
  RegCloseKey(key);
  return param_key;
}
#endif

SerialError serial_error(SerialPort* self){
  return self->err;
}

const char* serial_strerror(SerialError error){
  switch (error) {
    case SERIAL_ERR_NONE: 
      return "Operation succesfull";
    case SERIAL_ERR_DISCONNECT:
      return "Unexpectedly disconnected";
    case SERIAL_ERR_INVALID_DEVICE:
      return "Invalid device";
    case SERIAL_ERR_INVALID_BAUD:
      return "Invalid baud rate specified";
    case SERIAL_ERR_INVALID_CHAR_SIZE:
      return "Invalid charachter size specified";
    case SERIAL_ERR_INVALID_STOP_BITS:
      return "Invalid stop bits specified";
    case SERIAL_ERR_INVALID_TIMEOUT:
      return "Invalid timeout specified";
    case SERIAL_ERR_TIMEOUT:
      return "Operation timed out";
    case SERIAL_ERR_PLATFORM:
      return "A platform specific problem occured";
  }
  return "<invalid SerialError>";
}

bool serial_open_with_settings(SerialPort* self, const char* device, SerialSettings settings) {
  if(settings.baud_rate == 0) settings.baud_rate = 9600;
  if(settings.char_size == 0) settings.char_size = 8;
  if(settings.flow_control == 0) settings.flow_control = SERIAL_FLOWCTRL_NONE;
  if(settings.parity == 0) settings.parity = SERIAL_PARITY_NONE;
  if(settings.stop_bits == 0) settings.stop_bits = SERIAL_STOPBITS_ONE;
  else if(settings.timeout_ms < 0 
      && settings.timeout_ms != SERIAL_TIMEOUT_NONE
  ){
    self->err = SERIAL_ERR_INVALID_TIMEOUT;
    return false;
  }
  
#if SERIAL_OS_WINDOWS
  HANDLE handle = CreateFileA(
    device, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0
  );

  if (handle == INVALID_HANDLE_VALUE) {
    DWORD err = GetLastError();
    self->err = SERIAL_ERR_PLATFORM;
    return false;
  }
  DCB dcb;
  dcb.DCBlength = sizeof(dcb);
  if (!GetCommState(handle, &dcb)) {
    DWORD last_error = GetLastError();
    CloseHandle(handle);
    self->err = SERIAL_ERR_PLATFORM;
    return false;
  }

  // defaults
  dcb.fBinary = true;
  dcb.fNull = false;
  dcb.fAbortOnError = false;
  dcb.BaudRate = 0;
  dcb.ByteSize = 8;
  dcb.fOutxCtsFlow = false;
  dcb.fOutxDsrFlow = false;
  dcb.fDsrSensitivity = false;
  dcb.fOutX = false;
  dcb.fInX= false;
  dcb.fRtsControl = DTR_CONTROL_DISABLE;
  dcb.fParity = false;
  dcb.Parity = NOPARITY;
  dcb.StopBits = ONESTOPBIT;
  if (!SetCommState(handle, &dcb)) {
    DWORD last_error = GetLastError();
    CloseHandle(handle);
    self->err = SERIAL_ERR_PLATFORM;
    return false;
  }
#elif SERIAL_OS_LINUX
  int handle = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (handle < 0) {
    self->err = SERIAL_ERR_PLATFORM;
    return false;
  }
#endif
  
  serial_reconfigure(self, settings);

  self->path = device;
  self->handle = handle;
  self->settings = settings;
  self->err = SERIAL_ERR_NONE;
  return true;
}

SerialSettings serial_get_settings(SerialPort* self){
  return self->settings;
}

bool serial_reconfigure(SerialPort* self, SerialSettings settings) {
#if SERIAL_OS_WINDOWS
  DCB dcb;
  dcb.DCBlength = sizeof(dcb);
  if (!GetCommState(sp.handle, &dcb)) {
    DWORD last_error = GetLastError();
    CloseHandle(sp.handle);
    self->err = SERIAL_ERR_PLATFORM;
    return false;
  }

  // baud
  dcb.BaudRate = settings.baud_rate;

  // flow
  dcb.fOutxCtsFlow = false;
  dcb.fOutxDsrFlow = false;
  dcb.fTXContinueOnXoff = true;
  dcb.fDtrControl = DTR_CONTROL_ENABLE;
  dcb.fDsrSensitivity = false;
  dcb.fOutX = false;
  dcb.fInX = false;
  dcb.fRtsControl = RTS_CONTROL_ENABLE;
  if (settings.flow_control == SERIAL_FLOWCTRL_SOFTWARE) {
    dcb.fOutX = true;
    dcb.fInX = true;
  } else if (settings.flow_control == SERIAL_FLOWCTRL_HARDWARE) {
    dcb.fOutxCtsFlow = true;
    dcb.fRtsControl = RTS_CONTROL_HANDSHAKE;
  }

  // parity
  switch (settings.parity) {
    case SERIAL_PARITY_NONE:
      dcb.fParity = false;
      dcb.Parity = NOPARITY;
      break;
    case SERIAL_PARITY_ODD:
      dcb.fParity = true;
      dcb.Parity = ODDPARITY;
      break;
    case SERIAL_PARITY_EVEN:
      dcb.fParity = true;
      dcb.Parity = EVENPARITY;
      break;
  }

  // stop bits
  switch (settings.stop_bits) {
    case SERIAL_STOPBITS_ONE:
      dcb.StopBits = ONESTOPBIT;
      break;
    case SERIAL_STOPBITS_ONE_POINT_FIVE:
      dcb.StopBits = ONE5STOPBITS;
      break;
    case SERIAL_STOPBITS_TWO:
      dcb.StopBits = TWOSTOPBITS;
      break;
  }

  // char size
  dcb.ByteSize = settings.char_size;

  if (!SetCommState(sp.handle, &dcb)) {
    DWORD last_error = GetLastError();
    CloseHandle(sp.handle);
    self->err = SERIAL_ERR_PLATFORM;
    return false;
  }
#elif SERIAL_OS_LINUX
  struct termios tty;
  int fd = self->handle;

  if (tcgetattr(fd, &tty) < 0) {
    self->err = SERIAL_ERR_PLATFORM;
    return false;
  }

  // set defaults
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_iflag |= IGNPAR;

  // raw mode
  cfmakeraw(&tty);

  // baud
  int br = 0;
  switch (settings.baud_rate) {
    case 0:       br = B0;       break;
    case 50:      br = B50;      break;
    case 75:      br = B75;      break;
    case 110:     br = B110;     break;
    case 134:     br = B134;     break;
    case 150:     br = B150;     break;
    case 200:     br = B200;     break;
    case 300:     br = B300;     break;
    case 600:     br = B600;     break;
    case 1200:    br = B1200;    break;
    case 1800:    br = B1800;    break;
    case 2400:    br = B2400;    break;
    case 4800:    br = B4800;    break;
    case 9600:    br = B9600;    break;
    case 19200:   br = B19200;   break;
    case 38400:   br = B38400;   break;
    case 57600:   br = B57600;   break;
    case 115200:  br = B115200;  break;
    case 230400:  br = B230400;  break;
    case 460800:  br = B460800;  break;
    case 500000:  br = B500000;  break;
    case 576000:  br = B576000;  break;
    case 921600:  br = B921600;  break;
    case 1000000: br = B1000000; break;
    case 1152000: br = B1152000; break;
    case 1500000: br = B1500000; break;
    case 2000000: br = B2000000; break;
    case 2500000: br = B2500000; break;
    case 3000000: br = B3000000; break;
    case 3500000: br = B3500000; break;
    case 4000000: br = B4000000; break;
    default:
      self->err = SERIAL_ERR_INVALID_BAUD;
      return false;
  }
  cfsetospeed(&tty, br);
  cfsetispeed(&tty, br);

  // char size
  if (settings.char_size < 5 || settings.char_size > 8) {
    self->err = SERIAL_ERR_INVALID_CHAR_SIZE;
    return false;
  }
  switch (settings.char_size) {
    case 5:
      tty.c_cflag |= CS5;
      break;
    case 6:
      tty.c_cflag |= CS6;
      break;
    case 7:
      tty.c_cflag |= CS7;
      break;
    case 8:
      tty.c_cflag |= CS8;
      break;
  }

  // flow control
  switch (settings.flow_control) {
    case SERIAL_FLOWCTRL_NONE:
      tty.c_iflag &= ~(IXOFF | IXON);
      tty.c_cflag &= ~CRTSCTS;
      break;
    case SERIAL_FLOWCTRL_SOFTWARE:
      tty.c_iflag |= IXOFF | IXON;
      tty.c_cflag &= ~CRTSCTS;
      break;
    case SERIAL_FLOWCTRL_HARDWARE:
      tty.c_iflag &= ~(IXOFF | IXON);
      tty.c_cflag |= CRTSCTS;
      break;
  }

  // parity
  switch (settings.parity) {
    case SERIAL_PARITY_NONE:
      tty.c_iflag |= IGNPAR;
      tty.c_cflag &= ~(PARENB | PARODD);
      break;
    case SERIAL_PARITY_EVEN:
      tty.c_iflag &= ~(IGNPAR | PARMRK);
      tty.c_iflag |= INPCK;
      tty.c_cflag |= PARENB;
      tty.c_cflag &= ~PARODD;
      break;
    case SERIAL_PARITY_ODD:
      tty.c_iflag &= ~(IGNPAR | PARMRK);
      tty.c_iflag |= INPCK;
      tty.c_cflag |= (PARENB | PARODD);
      break;
  }

  // stop bits
  switch (settings.stop_bits) {
    case SERIAL_STOPBITS_ONE:
      tty.c_cflag &= ~CSTOPB;
      break;
    case SERIAL_STOPBITS_TWO:
      tty.c_cflag |= CSTOPB;
      break;
    case SERIAL_STOPBITS_ONE_POINT_FIVE:
      self->err = SERIAL_ERR_INVALID_STOP_BITS;
      return false;
  }

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    self->err = SERIAL_ERR_PLATFORM;
    return false;
  }
#endif
  self->err = SERIAL_ERR_NONE;
  return true;
}

bool serial_set_low_latency(SerialPort* self, bool ll) {
#if SERIAL_OS_WINDOWS
  int latency = 16;
  if (ll) {
    latency = 1;
  }
  HKEY key = serial_open_device_params(self->path);
  int r = RegSetValueEx(
    key, "LatencyTimer", 0, REG_DWORD, (LPBYTE)&latency, sizeof(latency)
  );
  if (r) {
    self->err = SERIAL_ERR_PLATFORM;
    return false;
  }
  RegCloseKey(key);
#elif SERIAL_OS_LINUX
  struct serial_struct ser;
  if(ioctl(self->handle, TIOCGSERIAL, &ser) < 0){
    self->err = SERIAL_ERR_PLATFORM;
    return false;
  }
  ser.flags |= ll ? ASYNC_LOW_LATENCY : ~ASYNC_LOW_LATENCY;
#endif
  self->err = SERIAL_ERR_NONE;
  return true;
}

bool serial_in_waiting(SerialPort* self) {
#if SERIAL_OS_WINDOWS
  DWORD flags;
  COMSTAT comstat;
  if(ClearCommError(sp.handle, &flags, &comstat) == false){
    self->err = SERIAL_ERR_PLATFORM;
    return false;
  }
  return comstat != 0;
#elif SERIAL_OS_LINUX
  int nbytes_ready = 0;
  if(ioctl(self->handle, FIONREAD, &nbytes_ready) < 0){
    self->err = SERIAL_ERR_PLATFORM;
    return false;
  }
  return nbytes_ready == 0;
#endif
}

void serial_clear_input(SerialPort* self) {
#if SERIAL_OS_WINDOWS
  PurgeComm(self->handle, PURGE_RXCLEAR | PURGE_RXABORT);
#elif SERIAL_OS_LINUX
  tcflush(self->handle, TCIFLUSH);
#endif
}

void serial_clear_output(SerialPort *self) {
#if SERIAL_OS_WINDOWS
  PurgeComm(self->handle, PURGE_TXCLEAR | PURGE_TXABORT);
#elif SERIAL_OS_LINUX
  tcflush(self->handle, TCOFLUSH);
#endif
}

int serial_read(SerialPort* self, char* buf, int length) {
  int timeout = self->settings.timeout_ms;
#if SERIAL_OS_WINDOWS
  COMMTIMEOUTS timeouts;
  if (timeout == SERIAL_TIMEOUT_INFINITE) {
    timeouts.ReadIntervalTimeout = 0;
    timeouts.ReadTotalTimeoutConstant = 0;
    timeouts.ReadTotalTimeoutMultiplier = 0;
  } else if (timeout == SERIAL_TIMEOUT_NONE) {
    timeouts.ReadIntervalTimeout = MAXDWORD;
    timeouts.ReadTotalTimeoutConstant = 0;
    timeouts.ReadTotalTimeoutMultiplier = 0;
  } else if (timeout > 0) {
    timeouts.ReadIntervalTimeout = 0;
    timeouts.ReadTotalTimeoutConstant = (DWORD)timeout;
    timeouts.ReadTotalTimeoutMultiplier = 0;
  }
  if (!SetCommTimeouts(self->handle, &timeouts)) {
    DWORD last_error = GetLastError();
    CloseHandle(self->handle);
    self->err = SERIAL_ERR_PLATFORM;
    return -1;
  }
  DWORD bytes_read = 0;
  if(ReadFile(self->handle, buf, length, &bytes_read, NULL) == false){
    self->err = SERIAL_ERR_PLATFORM;
    return -1;
  }else{
    self->err = SERIAL_ERR_NONE;
    return bytes_read;
  }
#elif SERIAL_OS_LINUX
  fd_set rfds;
  FD_ZERO(&rfds);
  FD_SET(self->handle, &rfds);

  struct timeval tv;
  tv.tv_sec = timeout / 1000;
  tv.tv_usec = (timeout % 1000) / 1000;

  struct timeval* tvp = &tv;
  if (timeout == SERIAL_TIMEOUT_INFINITE) {
    tvp = NULL;
  } else if (timeout == SERIAL_TIMEOUT_NONE) {
    tv.tv_sec = 0;
    tv.tv_usec = 0;
  }

  int read_count = 0;

  while (read_count < length) {
    int ready = select(self->handle + 1, &rfds, NULL, NULL, tvp);
    if (ready == -1) {
      self->err = SERIAL_ERR_PLATFORM;
      return -1;
    } else if (ready == 0) {
      self->err = SERIAL_ERR_TIMEOUT;
      return 0;
    }
    if (FD_ISSET(self->handle, &rfds) != 0) {
      int bytes_read = read(self->handle, buf, length - read_count);
      if (bytes_read < 0) {
        // error out
      }
      read_count += bytes_read;
    } else {
      // select finished, but the sp fd wasn't part of it
      // useful for cancelling, but impossible without a
      // special fd for cancellation.
      break;
    }
  }
  self->err = SERIAL_ERR_NONE;
  return read_count;
#endif
}

int serial_write(SerialPort* self, const char* buf, int length){
  int timeout = self->settings.timeout_ms;

#if SERIAL_OS_WINDOWS
  COMMTIMEOUTS timeouts;
  if (timeout == SERIAL_TIMEOUT_INFINITE) {
    timeouts.WriteTotalTimeoutConstant = MAXDWORD;
    timeouts.WriteTotalTimeoutMultiplier = 0;
  } else if (timeout == SERIAL_TIMEOUT_NONE) {
    timeouts.WriteTotalTimeoutConstant = 0;
    timeouts.WriteTotalTimeoutMultiplier = 0;
  } else if (timeout > 0) {
    timeouts.WriteTotalTimeoutConstant = (DWORD)timeout;
    timeouts.WriteTotalTimeoutMultiplier = 0;
  }

  if (!SetCommTimeouts(self->handle, &timeouts)) {
    DWORD last_error = GetLastError();
    CloseHandle(self->handle);
    self->err = SERIAL_ERR_PLATFORM;
    return false;
  }

  DWORD written_count = 0;
  bool status = WriteFile(self->handle, buf, length, &written_count, NULL);
#elif SERIAL_OS_LINUX
  if (timeout == 0) {
    int wlen = write(self->handle, buf, length);
    if (wlen != length) {
      // error out
      return -1;
    }
    return wlen;
  }
  fd_set wfds;
  FD_ZERO(&wfds);
  FD_SET(self->handle, &wfds);

  struct timeval tv;
  tv.tv_sec = timeout / 1000;
  tv.tv_usec = (timeout % 1000) / 1000;

  struct timeval* tvp = &tv;
  if (timeout == SERIAL_TIMEOUT_INFINITE) {
    tvp = NULL;
  }else if(timeout == SERIAL_TIMEOUT_NONE){
    tv.tv_sec = 0;
    tv.tv_usec = 0;
  }

  int written_count = 0;

  while (written_count < length) {
    int bytes_written = write(
      self->handle,
      &buf[written_count],
      length - written_count
    );

    int ready = select(self->handle + 1, NULL, &wfds, NULL, tvp);
    if (ready == -1) {
      // error
    } else if (ready == 0) {
      self->err = SERIAL_ERR_TIMEOUT;
      return written_count;
    }
    if (FD_ISSET(self->handle, &wfds)) {
      written_count += bytes_written;
    } else {
      // cancelled
      return 0;
    }
  }
#endif
  self->err = SERIAL_ERR_NONE;
  return written_count;
}
#endif // SERIAL_IMPLEMENTATION

#endif
