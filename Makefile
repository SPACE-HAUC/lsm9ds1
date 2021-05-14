CC=gcc
CXX=g++
RM= /bin/rm -vf
PYTHON=python3
ARCH=UNDEFINED
PWD=pwd
CDR=$(shell pwd)

EDCFLAGS:=$(CFLAGS)
EDLDFLAGS:=$(LDFLAGS)
EDDEBUG:=$(DEBUG)

EDCFLAGS:= -Wall -std=gnu11 $(EDCFLAGS) $(EDDEBUG)
EDLDFLAGS:= -lm -lpthread $(EDLDFLAGS)

test: EDCFLAGS:= -O2 -DLSM9DS1_UNIT_TEST $(EDCFLAGS)

BUILDDRV=drivers/i2cbus/i2cbus.o

BUILDOBJS=lsm9ds1.o \
$(BUILDDRV)

EPSTARGET=mag_tester.out

test: $(EPSTARGET)

$(EPSTARGET): $(BUILDOBJS)
	$(CC) $(BUILDOBJS) $(EDCFLAGS) -Idrivers/ -I./ $(LINKOPTIONS) -o $@ \
	$(EDLDFLAGS)

%.o: %.c
	$(CC) $(EDCFLAGS) $(EDDEBUG) -Idrivers/ -I./ -o $@ -c $<

.PHONY: clean

clean:
	$(RM) $(BUILDOBJS)
	$(RM) $(EPSTARGET)

spotless: clean

