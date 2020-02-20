CC=g++
NAME=test_ipopt
SRC=main.cpp nonlinprogram.cpp
OBJ=$(SRC:.cpp=.o)
CFLAGS=-Wall -Wextra
INCLUDES=-isystem /usr/include/eigen3/ -isystem ~/ipopt/include/coin/
LIBPATHS=-L ~/ipopt/lib
LIBS=-lipopt

all: $(NAME)

%.o: %.cpp
	$(CC) -c -o $@ $(CFLAGS) $< $(INCLUDES)

$(NAME): $(OBJ)
	$(CC) -o $(NAME) $(OBJ) $(LIBPATHS) $(LIBS)

clean:
	rm -f -- $(OBJ)

fclean: clean
	rm -f -- $(NAME)

re: fclean all

.PHONY: all clean fclean re
