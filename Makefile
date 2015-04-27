src = $(wildcard ./src/*.c)
obj = $(subst src,obj,$(src:.c=.o))

CFLAGS = -g -Wall -pedantic -Wno-deprecated-declarations
LDFLAGS = -L./lib/noise -framework audiotoolbox -framework corefoundation

audio: obj $(obj) libnoise
	$(CC) $(CFLAGS) -o $@ $(obj) -lnoise $(LDFLAGS)

obj:
	mkdir obj

obj/%.o: src/%.c
	$(CC) $(CFLAGS) -o $@ -c $<

.PHONY: libnoise
libnoise:
	@$(MAKE) -s -C ./lib/noise

.PHONY: clean print-% check-syntax 
clean:
	rm -f $(obj) audio

print-%:
	@echo $*=$($*)

check-syntax:
	$(CC) -o /dev/null -S $(CHK_SOURCES)
