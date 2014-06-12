MAVLINK_DIALECT = common
CFLAGS = -Wall -std=c99 -I./mavlink/$(MAVLINK_DIALECT)

all:
	gcc main.c $(CFLAGS) -o mavrate
