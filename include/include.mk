include $(TOPDIR)/config.mk

ifeq ($(TOPDIR),)
$(error TOPDIR value no set)
endif

inc_dirs :=

inc_files := $(strip $(foreach n, $(SUB_MODULES), \
			  $(patsubst %, $(n)/%, \
			    $(filter %.h, \
				  $(notdir $(shell ls $(TOPDIR)/$(n)/)) \
				) \
			  ) \
			))

inc_deps :=$(patsubst %, $(DIR)include/%, $(inc_files))
inc_dirs_deps :=$(strip $(foreach d, $(inc_dirs), \
			      $(patsubst %, $(DIR)include/%, $(notdir $(d))) \
			  ))

ifeq ($(V),1)
$(inc_dirs_deps):$(inc_dirs)
	@if [ ! -r $@ ]; then echo "    [GEN]    $@" && ln -s $(filter %/$(notdir $@),$(inc_dirs)) $@; fi;
else
$(inc_dirs_deps):$(inc_dirs)
	@if [ ! -r $@ ]; then echo "ln -s $(filter %/$(notdir $@),$(inc_dirs)) $@" && ln -s $(filter %/$(notdir $@),$(inc_dirs)) $@; fi;
endif

$(foreach s,$(inc_files),$(eval $(DIR)include/$(s):$(TOPDIR)/$(s)))

$(DIR)include/%:$(TOPDIR)/%
	@mkdir -p $(dir $@)
	$(call echocmd,GEN,$@,ln -s $< $@)
