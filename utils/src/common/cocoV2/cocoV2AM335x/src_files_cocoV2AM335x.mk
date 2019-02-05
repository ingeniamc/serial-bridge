
SRCDIR += src/cocoV2AM335x src/cocoV2AM335x/device src/cocoV2AM335x/include
INCDIR += src/cocoV2AM335x src/cocoV2AM335x/device src/cocoV2AM335x/include

# Common source files across all platforms and cores
SRCS_COMMON += cocoV2AM335x.c cocoV2AM335x_info.c cocoV2AM335x_lld_init.c cocoV2AM335x_ethernet_config.c cocoV2AM335x_pinmux.c enet_phy.c

PACKAGE_SRCS_COMMON =   src/cocoV2AM335x/src_files_cocoV2AM335x.mk
