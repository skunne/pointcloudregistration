CC=g++
NAME=test_ipopt
SRC=main.cpp nonlinprogram.cpp
OBJ=$(SRC:.cpp=.o)
CFLAGS=-Wall -Wextra
INCLUDES=-isystem /usr/include/pcl-1.9/ -isystem /usr/include/eigen3/ -isystem /usr/include/vtk/ -isystem ~/ipopt/include/coin/
LIBPATHS=-L/home/stephan/ipopt/lib
LIBS=-lvtkCommonDataModel -lvtkCommonMath -lvtkCommonCore -lvtkRenderingCore -lpcl_common -lpcl_filters -lpcl_io -lpcl_search -lpcl_sample_consensus -lpcl_visualization -lipopt

all: $(NAME)

%.o: %.cpp
	$(CC) -c -o $@ $(CFLAGS) $< $(INCLUDES)

$(NAME): $(OBJ)
	$(CC) -o $(NAME) $(OBJ) $(LIBPATHS) $(LIBS)

clean:
	rm -f $(OBJ)

fclean: clean
	rm -f $(NAME)

re: fclean all

.PHONY: all clean fclean re
