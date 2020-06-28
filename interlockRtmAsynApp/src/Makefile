TOP=../..

include $(TOP)/configure/CONFIG

USR_CXXFLAGS += -DHAVE_YAML

#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#========================================

INCLUDES += $(addprefix -I,$(YAML_INCLUDE))
INCLUDES += $(addprefix -I,$(BOOST_INCLUDE))

## this part is just for yaml loader driver
INCLUDES += $(addprefix -I,$(CPSW_FRAMEWORK_INCLUDE))

INCLUDES += $(addprefix -I,$(LLRFHLS_INCLUDE))

#INC += 

DBD += interlockRtmAsyn.dbd
DBD += timeAxisGenerator.dbd


LIBRARY_IOC += interlockRtmAsyn


interlockRtmAsyn_DBD  += interlockRtmAsyn.dbd
interlockRtmAsyn_DBD  += timeAxisGenerator.c
interlockRtmAsyn_SRCS += interlockRtmAsyn.cpp
interlockRtmAsyn_SRCS += timeAxisGenerator.c


cpsw_DIR = $(CPSW_FRAMEWORK_LIB)
llrfHls_DIR = $(LLRFHLS_LIB)

interlockRtmAsyn_LIBS += cpsw
interlockRtmAsyn_LIBS += llrfHls
interlockRtmAsyn_LIBS += asyn
interlockRtmAsyn_LIBS += yamlLoader
interlockRtmAsyn_LIBS += $(EPICS_BASE_IOC_LIBS)


#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
#

