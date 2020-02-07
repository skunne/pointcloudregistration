PYTHON=/usr/bin/python3
SCRIPTS=../../scripts_python/makepointcloud
DIGHOLE=$(PYTHON) $(SCRIPTS)/dig_hole.py
THINCLOUD=$(PYTHON) $(SCRIPTS)/thin_cloud.py
ROTATE=$(PYTHON) $(SCRIPTS)/apply_transform.py
SHUFFLE=$(PYTHON) $(SCRIPTS)/shuffle.py

DISTANCES=$(PYTHON) ../../scripts_python/other/distance_matched_pointclouds.py
AVG_DIST_FILE=average_distances.txt
# define COMPUTE_AVG_DISTANCES
# echo '' > $(AVG_DIST_FILE)
# for pc in matched_pointclouds/*_src.csv ; do \
# 	echo $$pc $${pc//_src.csv/_dst.csv} >> $(AVG_DIST_FILE) ; \
# 	$(DISTANCES) $$pc $${pc//_src.csv/_dst.csv} >> $(AVG_DIST_FILE) ; \
# done
# endef
define COMPUTE_AVG_DISTANCES
echo -n '' > $(AVG_DIST_FILE)
$(foreach pc,$(wildcard matched_pointclouds/*_src.csv),
echo $(pc) $({pc:_src.csv=_dst.csv) >> $(AVG_DIST_FILE)
$(DISTANCES) $(pc) $(pc:_src.csv=_dst.csv) >> $(AVG_DIST_FILE)
)
endef

RUNTESTS=../algo/run_tests
NAME=pointclouds/nuclei
PC1=$(NAME)_cropped.pcd $(NAME)_thinned.pcd $(NAME)_cropped_thinned.pcd
PC2=$(PC1) $(NAME)_rotated.pcd $(PC1:.pcd=_rotated.pcd)
POINTCLOUDS=$(NAME)_shuffled.pcd $(PC2)

ECCLEMJAR=~/.m2/repository/fr/univ-nantes/ec_clem/2.0.0-SNAPSHOT/ec_clem-2.0.0-SNAPSHOT.jar
CLITOOLSDIR=cli_tools
COMPUTETRANSFORMDIR=cli_tools/compute_transformation/
COMPUTETRANSFORMJAR=$(COMPUTETRANSFORMDIR)/target/compute_transformation-0.1.0-SNAPSHOT.jar
COMPUTETRANSFORM=java -jar $(COMPUTETRANSFORMJAR) --source-dataset $(1) --target-dataset $(2) --transformation-model RIGID

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

$(ECCLEMJAR):
	git clone https://github.com/anrcrocoval/ec-clem.git
	mvn -f ecclem/ clean install

$(CLITOOLSDIR):
	git clone https://github.com/anrcrocoval/cli_tools.git

$(COMPUTETRANSFORMJAR): $(CLITOOLSDIR) $(ECCLEMJAR)
	mvn -f $(COMPUTETRANSFORMDIR) clean install

$(NAME): $(RUNTESTS) $(POINTCLOUDS) $(COMPUTETRANSFORMJAR)
	mkdir -p matched_pointclouds
	$(RUNTESTS) $(NAME).meta $(NAME).pcd $(NAME).pcd $(POINTCLOUDS)
	$(COMPUTE_AVG_DISTANCES)
	$(call $(COMPUTETRANSFORM),matched_pointclouds/nuclei.pcd_src.csv,matched_pointclouds/nuclei.pcd_dst.csv)

clean:
	rm -f $(addprefix pointclouds/*,_shuffled.pcd _cropped.pcd _thinned.pcd _rotated.pcd)

fclean: clean
	rm -f $(RUNTESTS)

re: fclean all

.PHONY: all clean fclean re
