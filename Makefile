CFLAGS = -g -I src -I lib/minimp3/
LDLIBS = -lpulse -lpulse-simple

all: mplay

clean:
	rm -f mplay

mplay: src/mplay.c
	$(CC) -o $@ $^ $(CFLAGS) $(LDLIBS)

install:
	install -m755 -t mplay "$(DESTDIR)$(PREFIX)$(BINDIR)"

uninstall:
	rm -f "$(DESTDIR)$(PREFIX)$(BINDIR)/mplay"

.PHONY: all clean install uninstall
