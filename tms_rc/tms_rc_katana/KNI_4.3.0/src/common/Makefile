#===========================================================================
# KNI common Makefile
# copyright Neuronics Switzerland 2005-2008
# Authors: see AUTHORS file in the KNI root directory 
#===========================================================================

CXX		=	g++
CXXFLAGS	+=	-g -O2 -Wall -fPIC

TARGETS		=	alib

LIBNAME = knicommon
SOURCES = Timer.cpp

LIBNAME_A  = $(LIBNAME).a

INCLUDE_DIR = ../../include
TARGET_DIR  = .

.SILENT:

.SUFFIXES: .cpp.o

.cpp.o:
	echo "Compiling: $@"
	$(CXX) $(CXXFLAGS) -I$(INCLUDE_DIR) -c $< 
	

all: $(TARGETS)

$(TARGET_DIR)/$(LIBNAME_A): $(SOURCES:.cpp=.o)
	echo "Building: $@"
	$(AR) $(ARFLAGS) $@ $(SOURCES:.cpp=.o) > /dev/null
	

alib: $(TARGET_DIR)/$(LIBNAME_A)


clean:	
	echo "Cleaning up"
	rm -rf *.o *~  $(TARGET_DIR)/$(LIBNAME_A)
