
DIR_SRC = ./src
DIR_OBJ = ./obj
DIR_BIN = ./bin

DIR_SPRT_SRC=./spirit1_library/src

SRC =  $(wildcard  ${DIR_SRC}/*.c)
SRC += $(wildcard  ${DIR_SPRT_SRC}/*.c)
OBJ = $(patsubst %.c,${DIR_OBJ}/%.o,$(notdir ${SRC}))


INCLUDES += -I./inc
INCLUDES += -I./spirit1_library/inc


TARGET = spirit1_test

BIN_TARGET = ${DIR_BIN}/${TARGET}

CC=$(CXX)
CFLAGS = -g -Wall ${INCLUDES}

${BIN_TARGET}:${OBJ}
	$(CC) $(SYS_ROOT_PATH)  -g -lm -lpthread $(OBJ)  -o $@


${DIR_OBJ}/%.o:${DIR_SRC}/%.c
	$(CC) $(CFLAGS)  -c  -lpthread $< -o $@

${DIR_OBJ}/%.o:${DIR_SPRT_SRC}/%.c
	$(CC) $(CFLAGS) -c  $< -o $@

clean:
	rm -rf $(DIR_OBJ)/*.o
	rm -rf $(DIR_BIN)/*

dir:
	mkdir -p $(DIR_OBJ)
	mkdir -p $(DIR_BIN)

all:dir ${BIN_TARGET} 

	

