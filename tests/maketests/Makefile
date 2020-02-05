PYTHON = /usr/bin/python3
SCRIPTS = ../../scripts_python/makepointcloud
DIGHOLE = $(PYTHON) $(SCRIPTS)/dig_hole.py
THINCLOUD = $(PYTHON) $(SCRIPTS)/thincloud.py
ROTATE = $(PYTHON) $(SCRIPTS)/apply_transform.py
RUNTESTS = ../algo/run_tests
NAME = allo
PC1=$(NAME)_cropped.pcd $(NAME)_thinned.pcd $(NAME)_cropped_thinned.pcd
POINTCLOUDS= $(PC1) $(NAME)_rotated.pcd $(PC1:.pcd=_rotated.pcd)

all: $(NAME)

$(RUNTESTS):
	make -C ../algo

%_cropped.pcd: %.pcd
	$(DIGHOLE) $< $@ 5 5 5 10 10 10

%_thinned.pcd: %.pcd
	$(THINCLOUD) $< $@

%_rotated.pcd: %.pcd
	$(ROTATE) $< $@

$(NAME): $(POINTCLOUDS) $(RUNTESTS)
	$(RUNTESTS) $(NAME).meta $(NAME).pcd $(NAME).pcd $(POINTCLOUDS)
