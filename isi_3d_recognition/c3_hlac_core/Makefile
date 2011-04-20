ARCH = $(shell uname -m)
ifeq ($(ARCH), x86_64)
LIBRARY_URL = http://www.isi.imi.i.u-tokyo.ac.jp/~kanezaki/c3_hlac/pcl/x64/libc3_hlac_core.so
else
LIBRARY_URL = http://www.isi.imi.i.u-tokyo.ac.jp/~kanezaki/c3_hlac/pcl/x86/libc3_hlac_core.so
endif

$(shell mkdir -p $(shell rospack find c3_hlac_core)/lib)
ifeq ($(shell bash -c 'test -f $(shell rospack find c3_hlac_core)/lib/libc3_hlac_core.so && echo 1 || echo 0'), 0)
$(shell wget $(LIBRARY_URL) -P $(shell rospack find c3_hlac_core)/lib/)
endif

include $(shell rospack find mk)/cmake.mk

clean: wipe

wipe:
	rm -rf lib
