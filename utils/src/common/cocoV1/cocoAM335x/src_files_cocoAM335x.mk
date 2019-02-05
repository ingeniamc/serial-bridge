
SRCDIR += src/cocoAM335x src/cocoAM335x/device src/cocoAM335x/include
INCDIR += src/cocoAM335x src/cocoAM335x/device src/cocoAM335x/include

# Common source files across all platforms and cores
SRCS_COMMON += cocoAM335x.c cocoAM335x_info.c cocoAM335x_lld_init.c cocoAM335x_ethernet_config.c cocoAM335x_pinmux.c enet_phy.c

PACKAGE_SRCS_COMMON =   src/cocoAM335x/src_files_cocoAM335x.mk
