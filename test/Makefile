TESTS = rfm12b_read rfm12b_write rfm12b_echo rfm12b_chat
SRC = common/common.c
OBJ = $(SRC:.c=.o)

SRC_PROG = rfm12b_read/rfm12b_read.c rfm12b_write/rfm12b_write.c \
	rfm12b_echo/rfm12b_echo.c rfm12b_chat/rfm12b_chat.c
OBJ_PROG = $(SRC_PROG:.c=.o)

dir=@mkdir -p bin

all: $(TESTS)

rfm12b_read: rfm12b_read/rfm12b_read.o $(OBJ)
	$(dir)
	gcc $< $(OBJ) -o bin/$@  

rfm12b_write: rfm12b_write/rfm12b_write.o $(OBJ)
	$(dir)
	gcc $< $(OBJ) -o bin/$@

rfm12b_echo: rfm12b_echo/rfm12b_echo.o $(OBJ)
	$(dir)
	gcc $< $(OBJ) -o bin/$@  
	
rfm12b_chat: rfm12b_chat/rfm12b_chat.o $(OBJ)
	$(dir)
	gcc $< $(OBJ) -o bin/$@      

.c.o:
	gcc -c $< -o $@    

clean:
	rm -f $(OBJ) $(OBJ_PROG)
	rm -rf bin