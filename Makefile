CFLAGS = -g -I src -I lib/minimp3/
LDLIBS = 

.PHONY: all clean

all: build/mplay

clean:
	rm -rf build

build:
	mkdir build

build/%.o: src/%.c src/%.h | build
	$(CC) -o $@ -c $< $(CFLAGS) $(LDLIBS)

build/mplay: src/main.c | build
	$(CC) -o $@ $^ $(CFLAGS) $(LDLIBS)

