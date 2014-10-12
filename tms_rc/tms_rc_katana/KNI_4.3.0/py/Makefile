#===========================================================================
# KNI Python interface Makefile
# copyright Neuronics Switzerland 2005-2008
# Authors: see AUTHORS file in the KNI root directory 
#===========================================================================
PYTHON_INCLUDE = /usr/include/python2.6

KNI_INCLUDE   = ../include
KNI_LIBPATH   =	../lib/linux
KNI_LIBRARIES = $(KNI_LIBPATH)/kni.a $(KNI_LIBPATH)/libKNIBase.a $(KNI_LIBPATH)/libKNI_InvKin.a $(KNI_LIBPATH)/libKNI_LM.a $(KNI_LIBPATH)/kni.a $(KNI_LIBPATH)/libKinematics.a

CXXFLAGS        +=      -ggdb -fPIC

all: _KNI_noswig

rebuild: _KNI

KNI_wrap.cxx: KNI.i
	swig -c++ -python KNI.i


_KNI_noswig:
	echo "Building: KNI Python wrapper"
	$(CXX) $(CXXFLAGS) -c -I $(PYTHON_INCLUDE) -I $(KNI_INCLUDE) KNI_wrap.cxx
	$(CXX) $(CXXFLAGS) -shared *.o -o _KNI.so $(KNI_LIBRARIES)
	
_KNI: KNI_wrap.cxx
	echo "Building: KNI Python wrapper with SWIG"
	$(CXX) $(CXXFLAGS) -c -I $(PYTHON_INCLUDE) -I $(KNI_INCLUDE) KNI_wrap.cxx
	$(CXX) $(CXXFLAGS) -shared *.o -o $@.so $(KNI_LIBRARIES)

clean:
	rm -f *.so *.o *~ *.pyc *.zip 

#===========================================================================
