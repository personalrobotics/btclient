### Makefile for Barrett Software (ripple)

# Be sure to edit config.mk to match your installation
include ../../config.mk
include ../../common.mk

OBJS = main.o
TARG = ripple

# Done with defines, now for the real work
default: ripple

ripple: ${OBJS}
	$(CC) -o ${TARG} ${OBJS} ${LDFLAGS}
  
clean:
	rm -f *.o ${TARG} 

system:
	$(MAKE) -C ../btsystem lib
	$(MAKE) -C ../btsystem install

wam:
	$(MAKE) -C ../btwam lib
	$(MAKE) -C ../btwam install
   
all: system wam ripple

