# 2012 Xgaz <xgazza @ inwind.it>
# License: GPL

ifndef SOURCEDIR
export SOURCEDIR := $(PWD)
endif

module= af9033 tua9001 mxl5007t tda18218 dvb-usb-af9035
dvb-usb-af9035-objs = af9035.o 
obj-m += dvb-usb-af9035.o af9033.o af9035.o mxl5007t.o tua9001.o tda18218.o

EXTRA_CFLAGS := -DDETACHED_TERRATEC_MODULES \
                -I$(KBUILD_SRC)/drivers/media/dvb/dvb-usb/ \
		-I$(KBUILD_SRC)/drivers/media/dvb/dvb-core/ \
		-I$(KBUILD_SRC)/drivers/media/dvb/frontends/ \
		-I$(KBUILD_SRC)/drivers/media/common/tuners/

KDIR := /lib/modules/$(shell uname -r)/build
KINS = /lib/modules

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

install:
	cp dvb-usb-af9035.ko $(KINS)/`uname -r`/kernel/drivers/media/dvb/dvb-usb/ && cp mxl5007t.ko $(KINS)/`uname -r`/kernel/drivers/media/common/tuners/ && cp tua9001.ko $(KINS)/`uname -r`/kernel/drivers/media/common/tuners/ && cp tda18218.ko $(KINS)/`uname -r`/kernel/drivers/media/common/tuners/ && cp af9033.ko $(KINS)/`uname -r`/kernel/drivers/media/dvb/frontends/
	depmod -a

clean::
	-rm -f  *.o  *.ko *.mod.c .*.o.cmd  .*.o.d  .*.ko.cmd Module.symvers Module.markers modules.order
	-rm -rf .tmp_versions

