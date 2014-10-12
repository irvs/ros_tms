#===========================================================================
# KNI C wrapper Makefile
# copyright Neuronics Switzerland 2005-2008
# Authors: see AUTHORS file in the KNI root directory 
#===========================================================================

CXX		=	g++
CXXFLAGS	+=	-fPIC -g -O2 -Wall

TARGETS		=	alib solib

LIBNAME = kni
SOURCES = kni_wrapper.cpp

ADDITIONAL_DEPS = ../common/Timer.o

LIBNAME_A  = $(LIBNAME).a
LIBNAME_SO = $(LIBNAME).so

SLCFLAGS=-fPIC
SLLDFLAGS=-shared

INCLUDE_DIR = ../../include
TARGET_DIR  = ../../lib/linux

.SILENT:

.SUFFIXES: .cpp.o

.cpp.o:
	echo "Compiling: $@"
	$(CXX) $(CXXFLAGS) -I$(INCLUDE_DIR) -c $< 

all: $(TARGETS)

$(TARGET_DIR)/$(LIBNAME_A): $(SOURCES:.cpp=.o)
	echo "Building: $@ "
	$(AR) $(ARFLAGS) $@ $(SOURCES:.cpp=.o) > /dev/null
	echo "Adding common/additional dependencies..."	
	$(AR) -r $@ $(ADDITIONAL_DEPS) > /dev/null

$(TARGET_DIR)/$(LIBNAME_SO): $(SOURCES:.cpp=.o)
	echo "Building: $@"
	$(CXX) $(CXXFLAGS) $(SLLDFLAGS) -o $@ $(SOURCES:.cpp=.o) $(ADDITITIONAL_DEPS)

alib: $(TARGET_DIR)/$(LIBNAME_A)

solib: $(TARGET_DIR)/$(LIBNAME_SO)

clean:	
	echo "Cleaning up"
	rm -rf *.o *~ $(TARGET_DIR)/$(LIBNAME_A) $(TARGET_DIR)/$(LIBNAME_SO)
