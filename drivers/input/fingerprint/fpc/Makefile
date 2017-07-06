#
# Makefile for the tablet drivers
#

# Multipart objects.

#ccflags-y := -DDEBUG

#obj-m	+= fpc102x.o
obj-$(CONFIG_INPUT_FPC102X)	+= fpc102x.o

fpc102x-objs := fpc1020_main.o fpc1020_common.o fpc1020_input.o fpc1020_capture.o fpc1020_regulator.o

obj-$(CONFIG_INPUT_FPC)	+= touch/

