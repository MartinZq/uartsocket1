TOPDIR:=$(CURDIR)
export TOPDIR

$(shell find -name "*[.c][ch]" | xargs chmod -x)
$(shell find -name "*_config" | xargs chmod -x)

include $(TOPDIR)/config.mk

TARGET_NAME:=uartsocket
TARGET:=$(addprefix $(DIR),$(TARGET_NAME))
export TARGET

INCLUDE +=-I$(TOPDIR)/include
ifneq ($(DIR),)
INCLUDE += -I$(TOPDIR)/$(DIR)include
endif

LDFLAGS:=-lpthread
export INCLUDE LDFLAGS

TARGET_DMACRO:=-DTARGET_NAME=\"$(TARGET_NAME)\"
include mconfig/uartsocket_config
export TARGET_DMACRO

define target_dependsrcs
TARGET_OBJS +=$(DIR)$(1)/built.o
$(DIR)$(1)/built.o:$(addprefix $(DIR),$(patsubst %.c,%.o,$(2)))
	$(call echocmd,LD, $$@, \
          $(TARGET_LD) -r -o $$@ $$^)
endef

TARGET_SOURCES:=main.c
SUB_DIRS:=$(strip $(patsubst $(TOPDIR)/%/,%, \
		$(dir $(shell find -L $(TOPDIR) -name "transconn.mk"))))
SUB_MODULES:=debug $(SUB_DIRS)
export SUB_MODULES

TARGET_OBJS:=$(filter %.o,$(addprefix $(DIR),$(patsubst %.c,%.o,$(TARGET_SOURCES))))
TARGET_OBJS+=$(filter %.o,$(addprefix $(DIR),$(patsubst %.cc,%.o,$(TARGET_SOURCES))))

$(foreach d, $(SUB_DIRS), \
    $(eval include $(d)/transconn.mk) \
    $(if $(TARGET_SRCS), \
    	$(eval $(call target_dependsrcs,$(d),$(patsubst %,$(d)/%,$(TARGET_SRCS)))) \
    )  \
)

ALL_HEARDS:=$(shell find -L $(patsubst %,$(TOPDIR)/%,$(SUB_MODULES)) -name *.h)

.PHONY:all clean help

all:$(TARGET)

include $(TOPDIR)/include/include.mk

$(TARGET):$(inc_deps) $(inc_dirs_deps) target_comshow $(TARGET_OBJS)
	$(call echocmd,TAR,$@, \
	  $(TARGET_CC) $(TARGET_DMACRO) $(INCLUDE) $(LDPATH) $(TARGET_LDPATH) -O2 -o $@ $(TARGET_OBJS) $(LDFLAGS) $(TARGET_LDFLAG))
	@$(TARGET_STRIP) $@

$(DIR)%.o:%.c $(ALL_HEARDS) mconfig/uartsocket_config
	@if [ ! -d "$(dir $@)" ]; then mkdir -p $(dir $@); fi;
	$(call echocmd,CC, $@, \
	  $(TARGET_CC) $(TARGET_DMACRO) $(INCLUDE) -O2 -o $@ -c $<)

$(DIR)%.o:%.cc $(ALL_HEARDS) mconfig/uartsocket_config
	@if [ ! -d "$(dir $@)" ]; then mkdir -p $(dir $@); fi;
	$(call echocmd,CXX,$@, \
	  $(TARGET_CXX) $(TARGET_DMACRO) $(INCLUDE) -O2 -o $@ -c $<)

target_comshow:
	@echo ===========================================================
	@echo **compile $(TARGET_NAME): $(TARGET_CC)
	@echo ===========================================================

alls:all
	make -C tests

clean:
	(find -name "*.[oa]" | xargs $(RM)) && $(RM) $(TARGET)
	$(RM) -r $(patsubst %/,$(DIR)include/%,debug/ $(dir $(shell ls */transconn.mk))) $(inc_dirs_deps) $(DIR)

distclean:clean
	make -C tests clean

help:
	@echo "help:"
	@echo "  prefix=[compiler]	\"Cross compiler prefix.\""
	@echo "  V=[1|99]		\"Default value is 1, and 99 show more details.\""
	@echo "  dir=<path>		\"binaries path.\""
