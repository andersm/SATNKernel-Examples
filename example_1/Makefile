CROSSPREF = sh-elf-
CC = $(CROSSPREF)gcc
AS = $(CROSSPREF)gcc
LD = $(CROSSPREF)gcc
SIZE = $(CROSSPREF)size
OBJCOPY = $(CROSSPREF)objcopy

LDSCRIPT = ldscript

SATNKERNDIR = ../../SATNKernel
SATNKERNSHDIR = ../../SATNKernel/SH
APPDIR = .
OBJDIR = obj

vpath %.c $(APPDIR):$(SATNKERNDIR):$(SATNKERNSHDIR)
vpath %.S $(APPDIR):$(SATNKERNSHDIR)

CFLAGS = -Wall -Werror -m2 -Os -fomit-frame-pointer -std=c99 -ffunction-sections \
		 -I$(SATNKERNDIR) -I$(SATNKERNSHDIR)
DEBUGFLAGS = -g

ASFLAGS = -Wa,--big -Wa,--isa=sh2 -I$(SATNKERNSHDIR)

LDFLAGS = -nostartfiles -Wl,--gc-sections -T $(LDSCRIPT)

LIBS	= -lgcc

TARGET  = example_1.elf

CSOURCES =	tn.c					\
			tn_dqueue.c				\
			tn_event.c				\
			tn_mem.c				\
			tn_mutex.c				\
			tn_sem.c				\
			tn_tasks.c				\
			tn_utils.c				\
			tn_port_sh.c			\
			tn_user.c

ASMSOURCES =	crt0.S				\
				tn_port_isr.S		\
				tn_port_gas.S

COBJECTS = $(patsubst %.c,$(OBJDIR)/%.o,$(CSOURCES))
ASMOBJECTS = $(patsubst %.S,$(OBJDIR)/%.o,$(ASMSOURCES))

$(OBJDIR)/%.o: %.c
	$(CC) -c $(CFLAGS) $(DEBUGFLAGS) $< -o $@

$(OBJDIR)/%.o: %.S
	$(AS) -c $(ASFLAGS) $< -o $@

$(OBJDIR)/$(TARGET) : $(ASMOBJECTS) $(COBJECTS)
	$(CC) $(LDFLAGS) $(COBJECTS) $(ASMOBJECTS) $(LIBS) -o $@
	$(OBJCOPY) -O binary $(OBJDIR)/$(TARGET) $(OBJDIR)/$(basename $(TARGET)).bin
	@$(SIZE) $@

.PHONY : all
all : $(OBJDIR)/$(TARGET)

.PHONY : clean
clean :
	-rm -f $(OBJDIR)/*.o $(OBJDIR)/*.elf $(OBJDIR)/*.bin

#
# end of makefile
#
