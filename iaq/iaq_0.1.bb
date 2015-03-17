DESCRIPTION = "Linux kernel driver for IAQ"
LICENSE = "GPLv2+"
LIC_FILES_CHKSUM = "file://COPYING;md5=1f6f1c0be32491a0c8d2915607a28f36"

inherit module
INHERIT += "netlink"
PR = "r0"
PV = "0.1"

SRC_URI = "file://Makefile \
file://iaq.c \
file://iaq.h \
file://COPYING \
"

#DEPENDS = "netlink"

S = "${WORKDIR}"

# The inherit of module.bbclass will automatically name module packages with
# "kernel-module-" prefix as required by the oe-core build environment.

do_clean(){
	make clean
}
