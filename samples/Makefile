#
# Makefile for libnids samples.
#
# Dug Song <dugsong@monkey.org>

srcdir		= .


CC		= gcc
CFLAGS		= -g -O2 -D_BSD_SOURCE
LDFLAGS		= 

PCAP_CFLAGS	= -I/usr/local/include
PCAPLIB		= -L/usr/local/lib -lpcap

LNET_CFLAGS	= -D_BSD_SOURCE -D__BSD_SOURCE -D__FAVOR_BSD -DHAVE_NET_ETHERNET_H
LNETLIB		= -lnet

LIBS_CFLAGS	= -I../src $(PCAP_CFLAGS) $(LNET_CFLAGS)
#LIBS		= -L../src -lnids $(PCAPLIB) $(LNETLIB) -lgthread-2.0 -lnsl 
LIBS		= -L../src -lnids $(PCAPLIB) $(LNETLIB) -lgthread-2.0 -lnsl 


.c.o:
	$(CC) -c $(CFLAGS) -I. $(LIBS_CFLAGS) $<

all:  printall 
static shared: all
#### modified  z 2014 2 21


printall: printall.o
	$(CC) -o $@ printall.o $(LDFLAGS) $(LIBS)
####


static shared install installshared:
	@true

clean:
	rm -f *.o *~ overflows printall sniff

# EOF
