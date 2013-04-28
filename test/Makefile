TESTS = rf12_read rf12_write rf12_echo
SRC = common/common.c
OBJ = $(SRC:.c=.o)

SRC_PROG = rf12_read/rf12_read.c rf12_write/rf12_write.c \
	rf12_echo/rf12_echo.c
OBJ_PROG = $(SRC_PROG:.c=.o)

dir=@mkdir -p bin

all: $(TESTS)

rf12_read: rf12_read/rf12_read.o $(OBJ)
	$(dir)
	gcc $< $(OBJ) -o bin/$@  

rf12_write: rf12_write/rf12_write.o $(OBJ)
	$(dir)
	gcc $< $(OBJ) -o bin/$@

rf12_echo: rf12_echo/rf12_echo.o $(OBJ)
	$(dir)
	gcc $< $(OBJ) -o bin/$@      

.c.o:
	gcc -c $< -o $@    

clean:
	rm -f $(OBJ) $(OBJ_PROG)
	rm -rf bin