# 2012 Xgaz <xgazza@inwind.it>
# License: GPL

SOURCEDIR := $(PWD)
KDIR = /usr/src/linux-headers-`uname -r`
KINS = /lib/modules

EXTRA_CFLAGS += -I$(SOURCEDIR)/include/ \
                -DDETACHED_TERRATEC_MODULES \
                -I$(KDIR)/drivers/media/dvb/dvb-usb/ \
                -I$(KDIR)/drivers/media/dvb/dvb-core/ \
                -I$(KDIR)/drivers/media/dvb/frontends/  

module= af9033 tua9001 mxl5007t tuner_tda18218 dvb-usb-af9035
dvb-usb-af9035-objs = af9035.o
obj-m +=  mxl5007t.o tua9001.o tuner_tda18218.o dvb-usb-af9035.o af9033.o

default:
	make -C $(KDIR) SUBDIRS=$(PWD) modules
 
install:
	cp dvb-usb-af9035.ko $(KINS)/`uname -r`/kernel/drivers/media/dvb/dvb-usb/ && cp mxl5007t.ko $(KINS)/`uname -r`/kernel/drivers/media/common/tuners/ && cp tua9001.ko $(KINS)/`uname -r`/kernel/drivers/media/common/tuners/ && cp tuner_tda18218.ko $(KINS)/`uname -r`/kernel/drivers/media/common/tuners/ && cp af9033.ko $(KINS)/`uname -r`/kernel/drivers/media/dvb/frontends/
	depmod -a

clean::
	-rm -f  *.o  *.ko *.mod.c .*.o.cmd  .*.o.d  .*.ko.cmd Module.symvers Module.markers modules.order
	-rm -rf .tmp_versions
