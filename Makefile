################################################################################
######################### User configurable parameters #########################
# filename extensions
CEXTS:=c
ASMEXTS:=s S
CXXEXTS:=cpp c++ cc

# probably shouldn't modify these, but you may need them below
ROOT=.
FWDIR:=$(ROOT)/firmware
BINDIR=$(ROOT)/bin
SRCDIR=$(ROOT)/src
INCDIR=$(ROOT)/include

WARNFLAGS+=
EXTRA_CFLAGS=
EXTRA_CXXFLAGS=

# Set to 1 to enable hot/cold linking
USE_PACKAGE:=1

# Add libraries you do not wish to include in the cold image here
# EXCLUDE_COLD_LIBRARIES:= $(FWDIR)/your_library.a
EXCLUDE_COLD_LIBRARIES:= 

# Set this to 1 to add additional rules to compile your project as a PROS library template
IS_LIBRARY:=1
# TODO: CHANGE THIS!
LIBNAME:=lib2654
VERSION:=1.0.0
# EXCLUDE_SRC_FROM_LIB= $(SRCDIR)/unpublishedfile.c
EXCLUDE_SRC_FROM_LIB+= $(SRCDIR)/main.cpp
EXCLUDE_SRC_FROM_LIB+= $(INCDIR)/main.h
EXCLUDE_SRC_FROM_LIB+= $(INCDIR)/auton.h
EXCLUDE_SRC_FROM_LIB+= $(INCDIR)/stateMachine/behaviors
EXCLUDE_SRC_FROM_LIB+= $(INCDIR)/stateMachine/state
# this line excludes opcontrol.c and similar files

# files that get distributed to every user (beyond your source archive) - add
# whatever files you want here. This line is configured to add all header files
# that are in the the include directory get exported
TEMPLATE_FILES=$(INCDIR)/**/*.h $(INCDIR)/**/*.hpp

.DEFAULT_GOAL=quick

################################################################################
################################################################################
########## Nothing below this line should be edited by typical users ###########
-include ./common.mk
