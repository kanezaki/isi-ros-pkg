SAMPLE_DATA_URL = http://www.isi.imi.i.u-tokyo.ac.jp/~kanezaki/c3_hlac/pcl/sample.pcd
$(shell mkdir -p $(shell rospack find c3_hlac)/data)
ifeq ($(shell bash -c 'test -f $(shell rospack find c3_hlac)/data/sample.pcd && echo 1 || echo 0'), 0)
$(shell wget $(SAMPLE_DATA_URL) -P $(shell rospack find c3_hlac)/data/)
endif

include $(shell rospack find mk)/cmake.mk