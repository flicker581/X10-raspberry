
CCFLAGS = -Wall -lrt
CC	= gcc
REMOVE	= rm -f
INSTALL = install

x10-spi: x10-spi.c cm11.c
	$(CC) $(CCFLAGS) -o $@ $^

all: x10-spi

clean:
	$(REMOVE) x10-spi

install:
	$(INSTALL) -m 755 -o root -g root x10-spi /usr/local/bin/
