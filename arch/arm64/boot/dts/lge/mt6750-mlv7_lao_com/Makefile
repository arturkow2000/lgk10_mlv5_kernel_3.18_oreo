ALL_DTS := $(notdir $(wildcard echo ./*.dts))
dtb-y += $(patsubst %.dts,%.dtb,$(ALL_DTS))

DTB_LIST := $(DTB_FILES)

targets += dtbs dtbs_install

dtbs: $(addprefix $(obj)/, $(DTB_LIST))
	$(Q)rm -f $(obj)/*.dtb

clean-files := *.dtb

