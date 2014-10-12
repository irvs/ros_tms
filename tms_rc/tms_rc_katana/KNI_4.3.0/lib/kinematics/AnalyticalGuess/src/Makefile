#===========================================================================
# AnalyticalGuess Makefile
# (C) Neuronics AG
# Jonas Haller 2008
#===========================================================================
MAJOR_VERSION	=	0.1
MINOR_VERSION	=	0

CXX		=	g++
CXXFLAGS	+=	-fPIC -g -O2 -Wall
INCLUDE_DIR = ../include
TARGET_DIR  = ../lib

LIBNAME = libAnalyticalGuess
LIB_STATIC  = $(LIBNAME).a
LIB_SO = $(LIBNAME).so.$(MAJOR_VERSION).$(MINOR_VERSION)
LDFLAGS=-fPIC
LLDFLAGS=-shared
LIBDIR =

SOURCES = kinematics.cpp KatanaKinematicsDecisionAlgorithms.cpp kinematics6M90G.cpp kinematics6M90T.cpp kinematics6M180.cpp
TARGETS	= alib solib

.SILENT:

.cpp.o:
	echo -n "Compiling: $@  "
	if $(CXX) $(CXXFLAGS) -I$(INCLUDE_DIR) -c $< 2>.clog; \
	then echo " [ok]"; \
	     cat .clog; \
	else echo " [failed]"; \
	     cat .clog; \
	fi

all: $(TARGETS)

$(TARGET_DIR)/$(LIB_STATIC): $(SOURCES:.cpp=.o) $(TARGET_DIR)
	echo -n "Building: $@  "
	if $(AR) $(ARFLAGS) $@ $(SOURCES:.cpp=.o) 1>.alog 2>.alog; \
	then echo " [ok]"; \
	else echo " [failed]"; \
	     cat .alog; \
	fi

$(TARGET_DIR)/$(LIB_SO): $(SOURCES:.cpp=.o) $(TARGET_DIR)
	echo -n "Building: $@  "
	if $(CXX) $(CXXFLAGS) $(LLDFLAGS) -o $@ $(SOURCES:.cpp=.o) 2>.slog; \
	then echo " [ok]"; \
	     cat .slog; \
	fi

alib: $(TARGET_DIR)/$(LIB_STATIC)

solib: $(TARGET_DIR)/$(LIB_SO)

clean:
	echo "cleaning up...: "
	rm -rf *.o *~ ./.?log ../include/*~ $(TARGET_DIR)/$(LIB_STATIC) $(TARGET_DIR)/$(LIB_SO)
	echo "...done "

$(TARGET_DIR):
	mkdir -p $(TARGET_DIR)


