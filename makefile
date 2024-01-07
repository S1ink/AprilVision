rwildcard = $(foreach d,$(wildcard $(addsuffix *,$(1))),$(call rwildcard,$(d)/,$(2)) $(filter $(subst *,%,$(2)),$(d)))

OPT ?= release
ifeq ($(filter $(OPT),release debug),)
$(error invalid 'OPT' option - choose from {release/debug})
endif
PATCH_V4L ?= 1
USE_CAMERASERVER ?= 0

CROSS_PREFIX := aarch64-bullseye-linux-gnu-
CXX := g++
STD := c++20

SRC_DIR := src
OBJ_DIR := obj
OUT_DIR := out
VS_DIR := VisionServer/lib-vs

NAME := charged-up-vision
VISION := $(OUT_DIR)/$(NAME)
STREAMING := $(OUT_DIR)/$(NAME)-raw
VISIONSERVER := $(VS_DIR)/out/libvs3407.so
PACKAGE_DEPS := $(VS_DIR)/lib/libtensorflowlite.so $(VS_DIR)/lib/libedgetpu.so $(VISIONSERVER)

# SRCS := $(call rwildcard,$(SRC_DIR)/,*.cpp *.c *.S *.s)
VISION_SRCS := src/chargedup.cpp
STREAMING_SRCS := src/chargedup-raw.cpp
VISION_OBJS := $(VISION_SRCS:$(SRC_DIR)/%=$(OBJ_DIR)/%.o)
STREAMING_OBJS := $(STREAMING_SRCS:$(SRC_DIR)/%=$(OBJ_DIR)/%.o)

DEFINES := -D"PATCH_LIFECAM_V4L_PROP_IDS=$(PATCH_V4L)" -D"ENABLE_CAMERASERVER=$(USE_CAMERASERVER)"

ifeq ($(OPT),release)
#release options for Compiling and Linking
COPTS := -O3 -D__RELEASE__ $(DEFINES)
LOPTS :=
else
#debug options for Compiling and Linking
COPTS := -g -Og -D__DEBUG__ $(DEFINES)
LOPTS := -g
endif

#preprocessor flags
CPPFLAGS := -pthread -I$(VS_DIR)/include -I$(VS_DIR)/src  -I$(SRC_DIR)/libpixy/include \
	-I$(VS_DIR)/additions -I$(VS_DIR)/include/opencv4 -MMD -MP
#compile flags
CFLAGS := -Wall -fpermissive
#flags for compiling assembly source
ASMFLAGS := -mcpu=cortex-a72 #-mfpu=neon-fp-armv8
#linker flags
LDFLAGS := -pthread -Wall -L$(VS_DIR)/out -L$(VS_DIR)/lib -L$(SRC_DIR)/libpixy/out \
	-Wl,--unresolved-symbols=ignore-in-shared-libs -Wl,-rpath=$$ORIGIN
#libs to link against - see complete list in VisionServer/lib-vs/makefile
# LDLIBS := -ltensorflowlite -ledgetpu -lopencv_gapi \
# 	-lopencv_highgui -lopencv_ml -lopencv_objdetect \
# 	-lopencv_photo -lopencv_stitching -lopencv_video \
# 	-lopencv_calib3d -lopencv_features2d -lopencv_dnn \
# 	-lopencv_flann -lopencv_videoio -lopencv_imgcodecs \
# 	-lopencv_imgproc -lopencv_core -lvs3407 -lcameraserver \
# 	-lntcore -lcscore -lwpiutil -lwpimath -lwpilibc -lopencv_aruco -lpigpio
LDLIBS := -lcscore -lntcore -lwpiutil -lwpilibc -lopencv_core -lopencv_imgproc
LDLIBS_VPROC := -lapriltag -lwpimath -lopencv_aruco -lopencv_calib3d -lvs3407
ifneq ($(USE_CAMERASERVER),0)
LDLIBS += -lcameraserver
endif


ifeq ($(OS),Windows_NT)
CXX := $(CROSS_PREFIX)$(CXX)
RM-R := del /s /Q
CP := copy
/ := \\

else
RM-R := rm -r
CP := cp
/ := /

endif


$(OBJ_DIR)/%.cpp.o : $(SRC_DIR)/%.cpp | $(OBJ_DIR)
	$(CXX) $(COPTS) -c -o $(OBJ_DIR)/$(@F) -std=$(STD) $(CPPFLAGS) $(CFLAGS) $<

$(OBJ_DIR)/%.c.o : $(SRC_DIR)/%.c | $(OBJ_DIR)
	$(CXX) $(COPTS) -c -o $(OBJ_DIR)/$(@F) -std=$(STD) $(CPPFLAGS) $(CFLAGS) $<

$(OBJ_DIR)/%.S.o : $(SRC_DIR)/%.S | $(OBJ_DIR)
	$(CXX) $(COPTS) -c -o $(OBJ_DIR)/$(@F) -std=$(STD) $(ASMFLAGS) $(CPPFLAGS) $(CFLAGS) $<

$(OBJ_DIR)/%.s.o : $(SRC_DIR)/%.s | $(OBJ_DIR)
	$(CXX) $(COPTS) -c -o $(OBJ_DIR)/$(@F) -std=$(STD) $(ASMFLAGS) $(CPPFLAGS) $(CFLAGS) $<


$(VISION): $(VISION_OBJS) | $(OUT_DIR)
	$(CXX) $(LOPTS) -o $@ $(LDFLAGS) $(foreach file,$(^F),$(OBJ_DIR)/$(file)) $(LDLIBS) $(LDLIBS_VPROC)

$(STREAMING): $(STREAMING_OBJS) | $(OUT_DIR)
	$(CXX) $(LOPTS) -o $@ $(LDFLAGS) $(foreach file,$(^F),$(OBJ_DIR)/$(file)) $(LDLIBS)

# $(VISIONSERVER):
# 	$(MAKE) -C $(VS_DIR)/lib-vs shared

$(OUT_DIR) $(OBJ_DIR):
	mkdir $@


.PHONY: vision streaming clean rebuild

vision: $(VISION)

streaming: $(STREAMING)

clean:
	$(RM-R) $(OBJ_DIR)
	$(RM-R) $(OUT_DIR)

rebuild: build | clean


-include $(OBJS:.o=.d)