PYTHON=/usr/bin/python3
SCRIPTS=../../scripts_python/makepointcloud
DIGHOLE=$(PYTHON) $(SCRIPTS)/dig_hole.py
THINCLOUD=$(PYTHON) $(SCRIPTS)/thincloud.py
ROTATE=$(PYTHON) $(SCRIPTS)/apply_transform.py
SHUFFLE=$(PYTHON) $(SCRIPTS)/shuffle.py
RUNTESTS=../algo/run_tests
NAME=pointclouds/nuclei
PC1=$(NAME)_cropped.pcd $(NAME)_thinned.pcd $(NAME)_cropped_thinned.pcd
PC2=$(PC1) $(NAME)_rotated.pcd $(PC1:.pcd=_rotated.pcd)
POINTCLOUDS=$(NAME)_shuffled.pcd $(PC2)

all: $(NAME)

$(RUNTESTS):
	make -C ../algo

%_shuffled.pcd: %.pcd
	$(SHUFFLE) $< $@

%_cropped.pcd: %.pcd
	$(DIGHOLE) $< $@ 5 5 5 10 10 10

%_thinned.pcd: %.pcd
	$(THINCLOUD) $< $@

%_rotated.pcd: %.pcd
	$(ROTATE) $< $@

$(NAME): $(RUNTESTS) $(POINTCLOUDS)
	mkdir -p registered_pointclouds
	$(RUNTESTS) $(NAME).meta $(NAME).pcd $(NAME).pcd $(POINTCLOUDS)

clean:
	rm -f $(addprefix pointclouds/*,_shuffled.pcd _cropped.pcd _thinned.pcd _rotated.pcd)

fclean: clean
	rm -f $(RUNTESTS)

re: fclean all

.PHONY: all clean fclean re
