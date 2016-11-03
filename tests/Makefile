TOPDIR := $(dir $(shell pwd))

$(shell find -name "*.[ch]" | xargs chmod -x)

include $(TOPDIR)/config.mk

INCLUDE+=
LDFLAGS:=-lpthread

.PHONY: all clean
all:serialtest socktest

serialtest:serial.c serial-exp.c
	$(TARGET_CC) -o $@ $^ $(INCLUDE) -w $(LDFLAGS)
	@$(TARGET_STRIP) $@

socktest:sock.c sock-exp.c
	$(TARGET_CC) -o $@ $^ $(INCLUDE) -w $(LDFLAGS)
	@$(TARGET_STRIP) $@

clean:
	$(RM) *.o serialtest socktest
