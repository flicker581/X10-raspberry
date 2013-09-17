
CCFLAGS = -Wall -lrt
CC	= gcc
REMOVE	= rm -f
INSTALL = install

x10send: x10send.c
	$(CC) $(CCFLAGS) -o $@ $^

all: x10send

clean:
	$(REMOVE) x10send

install:
	$(INSTALL) -m 755 -o root -g root x10send /usr/local/bin/
