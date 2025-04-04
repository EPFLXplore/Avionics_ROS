#include <fcntl.h> // For open()
#include <stdio.h>
#include <termios.h> // For terminal settings
#include <unistd.h>  // For read(), close()

#define DEVICE_PATH "/dev/cu.usbserial-110" // Replace with your actual device file

int main() {

  int fd = open(DEVICE_PATH, O_RDONLY | O_NOCTTY); // open the serial port file in read mode only
  // if fd = -1 -> error

  struct termios serial_settings; //<termios.h>, used for configuring terminal, (TTY) and serial port settings
  tcgetattr(fd, &serial_settings); // reads the current serial port settings into serial_settings

  // Set Baud Rate to 115200
  cfsetispeed(&serial_settings, B115200); // Set input speed
  cfsetospeed(&serial_settings, B115200); // Set ouput speed

  // Configure: 8 Data bits, No parity, 1 Stop bit (8N1)
  serial_settings.c_cflag &= ~PARENB; // No parity
  serial_settings.c_cflag &= ~CSTOPB; // 1 stop bit
  serial_settings.c_cflag &= ~CSIZE;
  serial_settings.c_cflag |= CS8; // 8 data bits

  serial_settings.c_cflag |=
      CREAD | CLOCAL; // Enable reading & ignore modem control lines

  tcsetattr(fd, TCSANOW, &serial_settings); // Apply the settings

  char buffer[256];

  while (1) {
    int bytes_read = read(fd, buffer, sizeof(buffer) - 1); // port to read, buffer to store the data in, max size of the incoming data
    if (bytes_read > 0) {
      buffer[bytes_read] = '\0'; // Null-terminate the string
      printf("Received: %s\n", buffer);
    }
  }

  close(fd);
  return 0;
}