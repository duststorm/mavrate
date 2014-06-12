#define _POSIX_C_SOURCE 199309L
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <stddef.h>

#include <time.h>

#include "mavlink.h"

// Store all of the metadata for every message. Now we can pretty-print incoming data
mavlink_message_info_t msg_metadata[256] = MAVLINK_MESSAGE_INFO;

// Track the last time a message was seen to give an estimate of that message's periodicity.
struct timespec lastSeen[256] = {};

long diff(const struct timespec *t1, const struct timespec *t2);

int main(int argc, char *argv[])
{
	// Make sure the user specifies a port
	if (argc != 2) {
		puts("Please specify a serial port to read from.\nUsage: mavrate SERIAL_PORT");
		return EXIT_FAILURE;
	}

	// Open the specified serial port.
	int fd = open(argv[1], O_RDWR | O_NOCTTY | O_NDELAY);
	// If opening fails, error out.
	if (fd == -1) {
		char x[128];
		sprintf(x, "Failed to open port: '%s'.\n", argv[1]);
		perror(x);
		return EXIT_FAILURE;
	}
	// Otherwise set port options.
	else {
		// Set reads to be non-blocking.
		fcntl(fd, F_SETFL, 0);

		// And set up a myriad of serial port options
		struct termios options;
		tcgetattr(fd, &options); // Pull the current options for a read-modify-write configuration.
		// Set the input & output baud rate to 115200
		cfsetispeed(&options, B115200);
		cfsetospeed(&options, B115200);

		// Disable all input and output processing	
		options.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
				    INLCR | PARMRK | INPCK | ISTRIP | IXON);
		options.c_oflag = 0;

		// Disable line processing
		options.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
		
		// Disable character processing
		options.c_cflag &= ~(CSIZE | PARENB);
		options.c_cflag |= CS8;

		// Set read() to timeout immediately and return 1 character minimum.
		options.c_cc[VMIN]  = 1;
		options.c_cc[VTIME] = 0;

		// Enable the receiver and set local mode.
		options.c_cflag |= CLOCAL | CREAD;
		
		// Finally write these new settings for the port.
		if (tcsetattr(fd, TCSANOW, &options) < 0) {
			perror("Failed to configure port.");
			return EXIT_FAILURE;
		}

		// Set reads to be non-blocking on stdin as well. Necessary to allow for user input.
		tcgetattr(STDIN_FILENO, &options);
		options.c_lflag &= ~ICANON;
		options.c_cc[VMIN] = 0; // Specify that no characters may be read by calls to read().
		options.c_cc[VTIME] = 0; // Set timeout for a read() to be immediate.	
	
		// Finally write these new settings for the port.
		if (tcsetattr(STDIN_FILENO, TCSANOW, &options) < 0) {
			perror("Failed to configure stdin for non-blocking reads.");
			return EXIT_FAILURE;
		}
	}

	char buffer[1024];
	while (true) {
		
                mavlink_message_t message;
                mavlink_status_t status;

		// Process any new serial input
		int numBytes = read(fd, buffer, sizeof(buffer));
		if (numBytes) {
			for (int i = 0; i < numBytes; ++i) {
                                uint8_t decodeState = mavlink_parse_char(0, (uint8_t)(buffer[i]), &message, &status);
				if (decodeState) {
					// Print the current message and its transmission period
                                        struct timespec t;
					clock_gettime(CLOCK_REALTIME, &t);
					struct timespec *lastMessageTime = &lastSeen[message.msgid];
					printf("%s, rate: %.3fHz\n", msg_metadata[message.msgid].name, 1.0f / (float)(diff(lastMessageTime, &t) / 1e9));
					clock_gettime(CLOCK_REALTIME, &lastSeen[message.msgid]);
				}
			}
		}
	}	

	// Finally close the serial port.
	close(fd);

	// And return success.
	return EXIT_SUCCESS;
}

/**
 * Calculate t2 - t1 and return that value in nanoseconds.
 */
long diff(const struct timespec *t1, const struct timespec *t2)
{
    // First calculate the offset in seconds
    long timeDiff = (t2->tv_sec - t1->tv_sec) * 1e9;

    // Then modify that by the offset in nanoseconds
    timeDiff += (t2->tv_nsec - t1->tv_nsec);

    return timeDiff;
}

