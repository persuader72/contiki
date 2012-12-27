CONTIKI_PROJECT = hello-world

ifeq ($(TARGET),cooja)
PROJECT_SOURCEFILES += cooja_infomem.c
endif

all: $(CONTIKI_PROJECT)

CONTIKI = ../..
include $(CONTIKI)/Makefile.include

