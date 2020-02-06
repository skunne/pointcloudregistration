PYTHON=/usr/bin/python3
SCRIPTS=../../scripts_python/makepointcloud
DIGHOLE=$(PYTHON) $(SCRIPTS)/dig_hole.py
THINCLOUD=$(PYTHON) $(SCRIPTS)/thin_cloud.py
ROTATE=$(PYTHON) $(SCRIPTS)/apply_transform.py
SHUFFLE=$(PYTHON) $(SCRIPTS)/shuffle.py

DISTANCES=$(PYTHON) ../../scripts_python/other/distance_matched_pointclouds.py
AVG_DIST_FILE=average_distances.txt
define COMPUTE_AVG_DISTANCES
echo '' > $(AVG_DIST_FILE)
for pc in matched_pointclouds/*_src.csv ; do \
	echo $$pc $${pc//_src.csv/_dst.csv} >> $(AVG_DIST_FILE) ; \
	$(DISTANCES) $$pc $${pc//_src.csv/_dst.csv} >> $(AVG_DIST_FILE) ; \
done
endef

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
	$(DIGHOLE) $< $@ 0 0 5 10 10 10

%_thinned.pcd: %.pcd
	$(THINCLOUD) $< $@

%_rotated.pcd: %.pcd
	$(ROTATE) $< $@

$(NAME): $(RUNTESTS) $(POINTCLOUDS)
	mkdir -p matched_pointclouds
	$(RUNTESTS) $(NAME).meta $(NAME).pcd $(NAME).pcd $(POINTCLOUDS)
	$(COMPUTE_AVG_DISTANCES)




clean:
	rm -f $(addprefix pointclouds/*,_shuffled.pcd _cropped.pcd _thinned.pcd _rotated.pcd)

fclean: clean
	rm -f $(RUNTESTS)

re: fclean all

.PHONY: all clean fclean re
