src = $(wildcard ./src/*.c)
obj = $(subst src,obj,$(src:.c=.o))

CFLAGS = -g -Wall -pedantic -Wno-deprecated-declarations -Wno-missing-braces
LDFLAGS = -L./lib/noise -L./lib/input -framework audiotoolbox -framework iokit -framework corefoundation

audio: obj $(obj) libnoise libinput
	$(CC) $(CFLAGS) -o $@ $(obj) -lnoise -linput $(LDFLAGS)

obj:
	mkdir obj

obj/%.o: src/%.c
	$(CC) $(CFLAGS) -o $@ -c $<

.PHONY: libnoise libinput
libnoise:
	@$(MAKE) -s -C ./lib/noise

libinput:
	@$(MAKE) -s -C ./lib/input

.PHONY: clean print-% check-syntax 
clean:
	rm -f $(obj) audio

print-%:
	@echo $*=$($*)

check-syntax:
	$(CC) -o /dev/null -S $(CHK_SOURCES)
