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
RANSAC=../ransac/apply_ransac
NAME=pointclouds/nuclei
PC_AUX1=$(NAME)_cropped.pcd $(NAME)_thinned.pcd $(NAME)_cropped_thinned.pcd
PC_AUX2=$(PC_AUX1) $(NAME)_rotated.pcd $(PC_AUX1:.pcd=_rotated.pcd)
POINTCLOUDS=$(NAME)_shuffled.pcd $(PC_AUX2)
MATCHED_WITNESS=matched_$(NAME)_dst.csv
MATCHED=$(foreach pc,$(POINTCLOUDS),matched_$(pc)_dst.csv)
INLIERS=$(subst _dst.csv,_inliers_dst.csv,$(MATCHED))
CT_AUX=$(foreach pc,$(POINTCLOUDS),$(subst pointclouds/,$(TRANSFORMDIR),$(pc))_computed.csv)
COMPUTED_TRANSFORMS=$(CT_AUX) $(subst _computed.csv,_inliers_computed.csv,$(CT_AUX))
MATRIX_DIFFERENCES=$(subst _computed.csv,_diff,$(COMPUTED_TRANSFORMS))


ECCLEMJAR=~/.m2/repository/fr/univ-nantes/ec_clem/2.0.0-SNAPSHOT/ec_clem-2.0.0-SNAPSHOT.jar
CLITOOLSDIR=cli_tools
COMPUTETRANSFORMDIR=cli_tools/compute_transformation/
COMPUTETRANSFORMJAR=$(COMPUTETRANSFORMDIR)/target/compute_transformation-0.1.0-SNAPSHOT.jar
COMPUTETRANSFORM=java -jar $(COMPUTETRANSFORMJAR) --source-dataset $(1) --target-dataset $(2) --transformation-model RIGID
TRANSFORMDIR=transforms/


all: $(MATRIX_DIFFERENCES)

$(RUNTESTS):
	make -C ../algo

$(RANSAC):
	make -C ../ransac

$(ECCLEMJAR):
	git clone https://github.com/anrcrocoval/ec-clem.git
	mvn -f ecclem/ clean install

$(CLITOOLSDIR):
	git clone https://github.com/anrcrocoval/cli_tools.git

$(COMPUTETRANSFORMJAR): $(CLITOOLSDIR) $(ECCLEMJAR)
	mvn -f $(COMPUTETRANSFORMDIR) clean install

%_shuffled.pcd: %.pcd
	$(SHUFFLE) $< $@
	mkdir -p $(TRANSFORMDIR)
	printf '1,0,0,0\n0,1,0,0\n0,0,1,0\n0,0,0,1' > $(subst pointclouds/,$(TRANSFORMDIR),$@)_true.csv

%_cropped.pcd: %.pcd
	$(DIGHOLE) $< $@ 0 0 5 10 10 10
	mkdir -p $(TRANSFORMDIR)
	printf '1,0,0,0\n0,1,0,0\n0,0,1,0\n0,0,0,1' >$(subst pointclouds/,$(TRANSFORMDIR),$@)_true.csv

%_thinned.pcd: %.pcd
	$(THINCLOUD) $< $@
	mkdir -p $(TRANSFORMDIR)
	printf '1,0,0,0\n0,1,0,0\n0,0,1,0\n0,0,0,1' > $(subst pointclouds/,$(TRANSFORMDIR),$@)_true.csv

%_rotated.pcd: %.pcd
	$(ROTATE) $< $@
	mkdir -p $(TRANSFORMDIR)
	printf -- '-0.15643446504023104,-0.9876883405951377,0,0\n0.9876883405951377,-0.15643446504023104,0,0\n0,0,1,0\n0,0,0,1' > $(subst pointclouds/,$(TRANSFORMDIR),$@)_true.csv

$(INLIERS): DST=$(subst _inliers_dst.csv,_dst.csv,$@)
%_inliers_dst.csv: $(MATCHED_WITNESS) $(RANSAC)
	$(RANSAC) $(subst _dst.csv,_src.csv,$(DST)) $(DST) $(subst _dst.csv,_src.csv,$@) $@

$(MATCHED_WITNESS): $(RUNTESTS) $(POINTCLOUDS) $(COMPUTETRANSFORMJAR)
	mkdir -p matched_pointclouds
	$(RUNTESTS) $(NAME).meta $(NAME).pcd $(NAME).pcd $(POINTCLOUDS)
	$(COMPUTE_AVG_DISTANCES)

$(COMPUTED_TRANSFORMS): DESTIN_POINTCLOUD=$(subst $(TRANSFORMDIR),matched_pointclouds/,$(subst _computed.csv,_dst.csv,$@))
$(COMPUTED_TRANSFORMS): SOURCE_POINTCLOUD=$(subst _dst.csv,_src.csv,$(DESTIN_POINTCLOUD))
transforms/%_inliers_computed.csv: matched_pointclouds/%_inliers_dst.csv
	$(call COMPUTETRANSFORM,$(SOURCE_POINTCLOUD),$(DESTIN_POINTCLOUD)) > $@
%_computed.csv: $(MATCHED_WITNESS)
	$(call COMPUTETRANSFORM,$(SOURCE_POINTCLOUD),$(DESTIN_POINTCLOUD)) > $@

%_diff: %_computed.csv
	$(COMPAREMATRICES) $(subst _inliers,,$(subst _diff,_computed.csv,$@)) $(subst _inliers,,$(subst _diff,_true.csv,$@)) | tee $@



clean:
	rm -f $(addprefix pointclouds/*,_shuffled.pcd _cropped.pcd _thinned.pcd _rotated.pcd)
	rm -f $(TRANSFORMDIR)/*_true.csv
	rm -f source.adj dest.adj

fclean: clean
	rm -f matched_pointclouds/*.csv
	rm -f $(TRANSFORMDIR)/*_computed.csv
	rm -f $(TRANSFORMDIR)/*_diff

ffclean: fclean
	rm -f $(RUNTESTS)
	rm -f $(RANSAC)

re: fclean all

rere: ffclean all

.PHONY: all $(NAME) clean fclean ffclean re rere
