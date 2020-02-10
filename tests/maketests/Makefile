PYTHON=/usr/bin/python3

SCRIPTS=../../scripts_python/
DIGHOLE=$(PYTHON) $(SCRIPTS)/dig_hole.py
THINCLOUD=$(PYTHON) $(SCRIPTS)/thin_cloud.py
ROTATE=$(PYTHON) $(SCRIPTS)/apply_transform.py
SHUFFLE=$(PYTHON) $(SCRIPTS)/shuffle.py
COMPAREMATRICES=$(PYTHON) $(SCRIPTS)/normofdifference_44matrix.py
DISTANCES=$(PYTHON) $(SCRIPTS)/distance_matched_pointclouds.py
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
MATCHED=matched_$(NAME)_dst.csv
PC1=$(NAME)_cropped.pcd $(NAME)_thinned.pcd $(NAME)_cropped_thinned.pcd
PC2=$(PC1) $(NAME)_rotated.pcd $(PC1:.pcd=_rotated.pcd)
POINTCLOUDS=$(NAME)_shuffled.pcd $(PC2)

ECCLEMJAR=~/.m2/repository/fr/univ-nantes/ec_clem/2.0.0-SNAPSHOT/ec_clem-2.0.0-SNAPSHOT.jar
CLITOOLSDIR=cli_tools
COMPUTETRANSFORMDIR=cli_tools/compute_transformation/
COMPUTETRANSFORMJAR=$(COMPUTETRANSFORMDIR)/target/compute_transformation-0.1.0-SNAPSHOT.jar
COMPUTETRANSFORM=java -jar $(COMPUTETRANSFORMJAR) --source-dataset $(1) --target-dataset $(2) --transformation-model RIGID
TRANSFORMDIR=transforms/


all: $(NAME)

$(RUNTESTS):
	make -C ../algo

%_shuffled.pcd: %.pcd
	$(SHUFFLE) $< $@
	mkdir -p $(TRANSFORMDIR)
	printf '1,0,0,0\n0,1,0,0\n0,0,1,0,\n0,0,0,1' > $(subst pointclouds/,$(TRANSFORMDIR),$@)_true.csv

%_cropped.pcd: %.pcd
	$(DIGHOLE) $< $@ 0 0 5 10 10 10
	mkdir -p $(TRANSFORMDIR)
	printf '1,0,0,0\n0,1,0,0\n0,0,1,0,\n0,0,0,1' >$(subst pointclouds/,$(TRANSFORMDIR),$@)_true.csv

%_thinned.pcd: %.pcd
	$(THINCLOUD) $< $@
	mkdir -p $(TRANSFORMDIR)
	printf '1,0,0,0\n0,1,0,0\n0,0,1,0,\n0,0,0,1' > $(subst pointclouds/,$(TRANSFORMDIR),$@)_true.csv

%_rotated.pcd: %.pcd
	$(ROTATE) $< $@
	mkdir -p $(TRANSFORMDIR)
	printf -- '-0.15643446504023104,-0.9876883405951377,0,0\n0.9876883405951377,-0.15643446504023104,0,0\n0,0,1,0\n0,0,0,1' > $(subst pointclouds/,$(TRANSFORMDIR),$@)_true.csv

$(ECCLEMJAR):
	git clone https://github.com/anrcrocoval/ec-clem.git
	mvn -f ecclem/ clean install

$(CLITOOLSDIR):
	git clone https://github.com/anrcrocoval/cli_tools.git

$(COMPUTETRANSFORMJAR): $(CLITOOLSDIR) $(ECCLEMJAR)
	mvn -f $(COMPUTETRANSFORMDIR) clean install

$(MATCHED): $(RUNTESTS) $(POINTCLOUDS) $(COMPUTETRANSFORMJAR)
	mkdir -p matched_pointclouds
	$(RUNTESTS) $(NAME).meta $(NAME).pcd $(NAME).pcd $(POINTCLOUDS)
	$(COMPUTE_AVG_DISTANCES)

$(foreach pc,$(POINTCLOUDS),$(subst pointclouds/,$(TRANSFORMDIR),$(pc))_computed.csv): SOURCE_POINTCLOUD=$(subst $(TRANSFORMDIR),matched_pointclouds/,$(subst _computed.csv,_src.csv,$@))
$(foreach pc,$(POINTCLOUDS),$(subst pointclouds/,$(TRANSFORMDIR),$(pc))_computed.csv): DESTIN_POINTCLOUD=$(subst _src.csv,_dst.csv,$(SOURCE_POINTCLOUD))
$(foreach pc,$(POINTCLOUDS),$(subst pointclouds/,$(TRANSFORMDIR),$(pc))_computed.csv): $(MATCHED)
	$(call COMPUTETRANSFORM,$(SOURCE_POINTCLOUD),$(DESTIN_POINTCLOUD)) > $@

$(NAME): $(foreach pc,$(POINTCLOUDS),$(subst pointclouds/,$(TRANSFORMDIR),$(pc))_computed.csv)
	$(foreach matrix,$^,$(COMPAREMATRICES) $(matrix) $(subst _computed.csv,_true.csv,$(matrix)); \
	)

clean:
	rm -f $(addprefix pointclouds/*,_shuffled.pcd _cropped.pcd _thinned.pcd _rotated.pcd)
	rm -f $(TRANSFORMDIR)/*_true.csv

fclean: clean
	rm -f matched_pointclouds/*.csv
	rm -f $(TRANSFORMDIR)/*_computed.csv

ffclean: fclean
	rm -f $(RUNTESTS)

re: fclean all

rere: ffclean all

.PHONY: all $(NAME) clean fclean ffclean re rere
