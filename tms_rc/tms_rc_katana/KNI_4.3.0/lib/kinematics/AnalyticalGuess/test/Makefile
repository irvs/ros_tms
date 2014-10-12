#===========================================================================
#
# Makefile
#
# Jonas Haller
#
#   Copyright (c) 2008
#
#   Neuronics AG <www.neuronics.ch>
#   Switzerland
#
#===========================================================================

CXX		=	g++
CXXFLAGS	+=	-g -O2 -Wall

TARGETS		=	kinematics
CTARGETS	=	keyboard

TARGETS_T	=	$(foreach target,$(TARGETS),$(target).test)
TARGETS_T_C	=	$(foreach target,$(TARGETS),$(target).test_clean)

CTARGETS_T	=	$(foreach target,$(CTARGETS),$(target).common)

INCLUDE_DIR = ../include -I./
LIBRARY_DIR  = ../lib

KINEMATICS_LIBRARY = $(LIBRARY_DIR)/libKatanaKinematics.a

AVAILABLE_LIBRARIES = ./keyboard.o ${KINEMATICS_LIBRARY}

.SILENT:

all: $(TARGETS_T)

$(CTARGETS_T):
	echo -n "Building: $@"
	if $(CXX) $(CXXFLAGS) -I$(INCLUDE_DIR) -c -o $(@:.common=).o $(wildcard $(@:.common=).cpp) 2>.clog; \
	then echo " [ok]"; \
		cat .clog; \
	else echo " [failed]"; \
		cat .clog; \
	fi

$(TARGETS_T): $(CTARGETS_T)
	echo -n "Building: $@"
	if $(CXX) $(CXXFLAGS) -I$(INCLUDE_DIR) -lpthread -o $(@:.test=) $(wildcard $(@:.test=).cpp) ${AVAILABLE_LIBRARIES} 2>.clog; \
	then echo " [ok]"; \
		cat .clog; \
	else echo " [failed]"; \
		cat .clog; \
	fi

$(TARGETS_T_C):
	echo "Cleaning: $(@:_clean=)"
	rm -f $(@:.test_clean=)

clean: $(TARGETS_T_C)
	echo "Cleaning up"
	rm -f *.o
	rm -f *~
	rm -f ./.?log
	rm -f ./*~
