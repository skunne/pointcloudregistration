CC=g++
NAME=apply_ransac
SRC=main_test.cpp
OBJ=$(SRC:.cpp=.o)
CFLAGS=-Wall -Wextra -Werror
INCLUDES=-isystem /usr/include/pcl-1.11/ -isystem /usr/include/eigen3/ -isystem /usr/include/vtk/
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
