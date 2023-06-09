CFLAGS = -g -I lib/minimp3/ -Wunused-variable
LDLIBS = -lpulse -lpulse-simple

PREFIX ?= /usr/local
BINDIR ?= /bin

all: mplay

clean:
	rm -f mplay

mplay: mplay.c
	$(CC) -o $@ $^ $(CFLAGS) $(LDLIBS)

install:
	install -m755 mplay -t "$(DESTDIR)$(PREFIX)$(BINDIR)"

uninstall:
	rm -f "$(DESTDIR)$(PREFIX)$(BINDIR)/mplay"

.PHONY: all clean install uninstall
