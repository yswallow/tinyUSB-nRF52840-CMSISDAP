include ../../../tools/top.mk
include ../../make.mk

INC += \
	src \
	src/DAP \
	$(TOP)/hw \

# $(NRF5_SDK_ROOT)/modules/nrfx/hal \
# Example source
EXAMPLE_SOURCE = \
	src/main.c \
	src/usb_descriptors.c \
	\
	src/dap_glue.c \

EXAMPLE_CPP = \
	src/DAP/board.cpp \
	src/DAP/DAP_config.cpp \
	src/DAP/dap_vendor_command.cpp \
	src/DAP/JTAG_DP.cpp \
	src/DAP/semihost.cpp \
	src/DAP/SW_DP.cpp \
	src/DAP/DAP.cpp \

SRC_C += $(addprefix $(CURRENT_PATH)/, $(EXAMPLE_SOURCE))
SRC_CPP += $(addprefix $(CURRENT_PATH)/, $(EXAMPLE_CPP))

include ../../rules.mk

NOCPPFLAGS := -Wstrict-prototypes -Werror-implicit-function-declaration 
CPPFLAGS := $(filter-out $(NOCPPFLAGS),$(CFLAGS))

ifeq ($(VERBOSE),1)
NO_ECHO := 
else
NO_ECHO := @
endif

vpath %.cpp . $(TOP)
$(BUILD)/obj/%.cpp.o: %.cpp
	@echo CC $(notdir $@)
	$(NO_ECHO)$(CXX) $(CPPFLAGS) -c -MD -o $@ $<
