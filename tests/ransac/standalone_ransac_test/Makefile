CC=g++
NAME=a.out
SRC=cpr_randomsampleconsensus.cpp
OBJ=$(SRC:.cpp=.o)
CFLAGS=-Wall -Wextra -Werror
INCLUDES=-I /usr/include/pcl-1.11/ -I /usr/include/eigen3/ -I /usr/include/vtk/
LIBS=-lvtkCommonDataModel -lvtkCommonMath -lvtkCommonCore -lvtkRenderingCore -lpcl_common -lpcl_filters -lpcl_io -lpcl_sample_consensus -lpcl_visualization

all: $(NAME)

%.o: %.cpp
	$(CC) -c -o $@ $(CFLAGS) $(SRC) $(INCLUDES)

$(NAME): $(OBJ)
	$(CC) -o $(NAME) $(OBJ) $(LIBS)

clean:
	rm -f -- $(OBJ)

fclean: clean
	rm -f -- $(NAME)

re: fclean all

.PHONY: all clean fclean re
