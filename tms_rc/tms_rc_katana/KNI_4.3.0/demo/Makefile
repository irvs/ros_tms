#===========================================================================
# KNI demo Makefile
# copyright Neuronics Switzerland 2005-2008
# Authors: see AUTHORS file in the KNI root directory 
#===========================================================================

CXX		=	g++
CXXFLAGS	+=	-g -O2 -Wall

TARGETS		=	kni_test control commands kni_wrapper wrapper_control mykni
CTARGETS	=	keyboard

TARGETS_D	=	$(foreach target,$(TARGETS),$(target).demo)
TARGETS_D_C	=	$(foreach target,$(TARGETS),$(target).demo_clean)

CTARGETS_D	=	$(foreach target,$(CTARGETS),$(target).common)

INCLUDE_DIR = ../include -I./common
LIBRARY_DIR  = ../lib/linux

BASE_LIBRARY = $(LIBRARY_DIR)/libKNIBase.a
KINE_LIBRARY = $(LIBRARY_DIR)/libKinematics.a
INVKIN_LIBRARY = $(LIBRARY_DIR)/libKNI_InvKin.a
LM_LIBRARY = $(LIBRARY_DIR)/libKNI_LM.a
WRAPPER_LIBRARY = $(LIBRARY_DIR)/kni.a

AVAILABLE_LIBRARIES = ./common/keyboard.o  ${WRAPPER_LIBRARY} ${LM_LIBRARY} ${INVKIN_LIBRARY} ${KINE_LIBRARY} ${BASE_LIBRARY} 

#.SILENT:

all: $(TARGETS_D)

$(CTARGETS_D):
	echo "Building: $@"
	$(CXX) $(CXXFLAGS) -I$(INCLUDE_DIR) -c -o common/$(@:.common=).o $(wildcard common/$(@:.common=).cpp)
	

$(TARGETS_D): $(CTARGETS_D)
	echo "Building: $@"
	$(CXX) $(CXXFLAGS) -I$(INCLUDE_DIR) -lpthread -o $(@:.demo=)/$(@:.demo=) $(wildcard $(@:.demo=)/*.cpp) ${AVAILABLE_LIBRARIES}
	

$(TARGETS_D_C):
	echo "Cleaning: $(@:_clean=)"
	rm -f $(@:.demo_clean=)/$(@:.demo_clean=)
	rm -f $(@:.demo_clean=)/*~
	rm -f $(@:.demo_clean=)/*.o

clean_common:
	echo "Cleaning: common"
	rm -f common/*.o
	rm -f common/*~

clean: clean_common $(TARGETS_D_C)
	echo "Cleaning up"
	rm -f ./.?log
