PYTHON=/usr/bin/python3

SCRIPTS=../../scripts_python/
DIGHOLE=$(PYTHON) $(SCRIPTS)/dig_hole.py
THINCLOUD=$(PYTHON) $(SCRIPTS)/thin_cloud.py
ROTATE=$(PYTHON) $(SCRIPTS)/apply_transform.py
SHUFFLE=$(PYTHON) $(SCRIPTS)/shuffle.py
COMPAREMATRICES=$(PYTHON) $(SCRIPTS)/normofdifference_44matrix.py
DISTANCES=$(PYTHON) $(SCRIPTS)/distance_matched_pointclouds_csv.py
PLOT_AVERAGE_DISTANCES=./average_distances_plot.py
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
ICPRANSAC=../icpransac/icpransac_gettransform
TESTCASE=heart
NAME=pointclouds/$(TESTCASE)

PC_AUX1=$(NAME).pcd $(NAME)_cropped.pcd $(NAME)_thinned.pcd $(NAME)_cropped_thinned.pcd
PC_AUX2=$(PC_AUX1) $(PC_AUX1:.pcd=_rotatedpi4.pcd) $(PC_AUX1:.pcd=_rotatedpi2.pcd)
POINTCLOUDS=$(PC_AUX2)
#POINTCLOUDS=$(NAME)_shuffled.pcd $(PC_AUX2)
#PC_AUX1=valve.pcd valve_r0.0.pcd valve_r0.3141592653589793.pcd valve_r0.6283185307179586.pcd valve_r0.9424777960769379.pcd valve_r1.2566370614359172.pcd valve_r1.5707963267948966.pcd
# POINTCLOUDS=$(addprefix pointclouds/,$(PC_AUX2))

MATCHED_WITNESS=matched_$(NAME)_dst.csv
MATCHED=$(foreach pc,$(POINTCLOUDS),matched_$(pc)_dst.csv)
INLIERS=$(subst _dst.csv,_inliers_dst.csv,$(MATCHED))
CT_AUX=$(foreach pc,$(POINTCLOUDS),$(subst pointclouds/,$(TRANSFORMDIR),$(pc))_computed.csv)
CT_AUX_INLIERS=$(subst _computed.csv,_inliers_computed.csv,$(CT_AUX))
COMPUTED_TRANSFORMS=$(CT_AUX) $(CT_AUX_INLIERS)
COMPUTED_TRANSFORMS_ICPRANSAC=$(subst _computed.csv,_icpransac.csv,$(CT_AUX))
MATRIX_DIFFERENCES=$(subst _computed.csv,_diff,$(COMPUTED_TRANSFORMS))

ifeq ($(TESTCASE),nuclei)
DIGCOORD=0 0 5 10 10 10
else
DIGCOORD=130 60 0 240 240 300
endif

ECCLEMDIR=ec-clem
ECCLEMJAR=~/.m2/repository/fr/univ-nantes/ec_clem/2.0.1-SNAPSHOT/ec_clem-2.0.1-SNAPSHOT.jar
CLITOOLSDIR=cli_tools
COMPUTETRANSFORMDIR=cli_tools/compute_transformation/
COMPUTETRANSFORMJAR=$(COMPUTETRANSFORMDIR)/target/compute_transformation-0.1.0-SNAPSHOT.jar
COMPUTETRANSFORM=java -jar $(COMPUTETRANSFORMJAR) --source-dataset $(1) --target-dataset $(2) --transformation-model RIGID
TRANSFORMDIR=transforms/


all: $(MATRIX_DIFFERENCES) $(COMPUTED_TRANSFORMS_ICPRANSAC) average_distances.png

compute_all_icpransac: $(COMPUTED_TRANSFORMS_ICPRANSAC)

$(RUNTESTS):
	make -C ../algo

$(RANSAC):
	make -C ../ransac

$(ECCLEMDIR):
	git clone https://github.com/anrcrocoval/ec-clem.git

$(ECCLEMJAR): $(ECCLEMDIR)
	-ln --symbolic ~/icy ec-clem/lib/icy
	mvn -f ec-clem/ clean
	mvn -f ec-clem install

$(CLITOOLSDIR):
	git clone https://github.com/anrcrocoval/cli_tools.git

$(COMPUTETRANSFORMJAR): $(CLITOOLSDIR) $(ECCLEMJAR)
	mvn -f $(COMPUTETRANSFORMDIR) clean
	mvn -f $(COMPUTETRANSFORMDIR) install

# %_shuffled.pcd: %.pcd
# 	$(SHUFFLE) $< $@
# 	mkdir -p $(TRANSFORMDIR)
# 	printf '1,0,0,0\n0,1,0,0\n0,0,1,0\n0,0,0,1' > $(subst pointclouds/,$(TRANSFORMDIR),$@)_true.csv
#

%_cropped.pcd: %.pcd
	$(DIGHOLE) $< $@ $(DIGCOORD)
	mkdir -p $(TRANSFORMDIR)
	printf '1,0,0,0\n0,1,0,0\n0,0,1,0\n0,0,0,1' >$(subst pointclouds/,$(TRANSFORMDIR),$@)_true.csv

%_thinned.pcd: %.pcd
	$(THINCLOUD) $< $@
	mkdir -p $(TRANSFORMDIR)
	printf '1,0,0,0\n0,1,0,0\n0,0,1,0\n0,0,0,1' > $(subst pointclouds/,$(TRANSFORMDIR),$@)_true.csv

%_rotatedpi4.pcd: %.pcd
	$(ROTATE) $< $@ 0.785 $(subst pointclouds/,$(TRANSFORMDIR),$@)_true.csv
	mkdir -p $(TRANSFORMDIR)
	#printf -- '-0.15643446504023104,-0.9876883405951377,0,0\n0.9876883405951377,-0.15643446504023104,0,0\n0,0,1,0\n0,0,0,1' > $(subst pointclouds/,$(TRANSFORMDIR),$@)_true.csv

%_rotatedpi2.pcd: %.pcd
	$(ROTATE) $< $@ 1.571 $(subst pointclouds/,$(TRANSFORMDIR),$@)_true.csv
	mkdir -p $(TRANSFORMDIR)
	#printf -- '-0.15643446504023104,-0.9876883405951377,0,0\n0.9876883405951377,-0.15643446504023104,0,0\n0,0,1,0\n0,0,0,1' > $(subst pointclouds/,$(TRANSFORMDIR),$@)_true.csv

# %.pcd:
# 	./buildrotatedpointclouds.sh

$(INLIERS): DST=$(subst _inliers_dst.csv,_dst.csv,$@)
%_inliers_dst.csv: $(MATCHED_WITNESS) $(RANSAC)
	$(RANSAC) $(subst _dst.csv,_src.csv,$(DST)) $(DST) $(subst _dst.csv,_src.csv,$@) $@

$(MATCHED_WITNESS): $(RUNTESTS) $(POINTCLOUDS) $(COMPUTETRANSFORMJAR)
	mkdir -p matched_pointclouds
	$(RUNTESTS) $(NAME).meta $(NAME).pcd $(POINTCLOUDS)
	$(COMPUTE_AVG_DISTANCES)

$(COMPUTED_TRANSFORMS): DESTIN_POINTCLOUD=$(subst $(TRANSFORMDIR),matched_pointclouds/,$(subst _computed.csv,_dst.csv,$@))
$(COMPUTED_TRANSFORMS): SOURCE_POINTCLOUD=$(subst _dst.csv,_src.csv,$(DESTIN_POINTCLOUD))
transforms/%_inliers_computed.csv: matched_pointclouds/%_inliers_dst.csv
	$(call COMPUTETRANSFORM,$(SOURCE_POINTCLOUD),$(DESTIN_POINTCLOUD)) > $@; true
%_computed.csv: $(MATCHED_WITNESS)
	$(call COMPUTETRANSFORM,$(SOURCE_POINTCLOUD),$(DESTIN_POINTCLOUD)) > $@; true

transforms/%_icpransac.csv: $(SOURCE_POINTCLOUD) $(DESTIN_POINTCLOUD)
	$(ICPRANSAC) $(NAME).pcd $(subst transforms/,pointclouds/,$(subst _icpransac.csv,,$@)) $@

%_diff: %_computed.csv
	$(COMPAREMATRICES) $(subst _inliers,,$(subst _diff,_computed.csv,$@)) $(subst _inliers,,$(subst _diff,_true.csv,$@)) | tee $@

average_distances.png: $(PLOT_AVERAGE_DISTANCES) $(COMPUTED_TRANSFORMS)
	$(PLOT_AVERAGE_DISTANCES)

clean:
	rm -f -- $(addprefix pointclouds/*,_shuffled.pcd _cropped.pcd _thinned.pcd _rotated.pcd)
	rm -f -- pointclouds/*_r*.pcd pointclouds/*_r*.csv
	rm -f -- $(TRANSFORMDIR)/*_true.csv
	rm -f -- source.adj dest.adj

fclean: clean
	rm -f -- matched_pointclouds/*.csv
	rm -f -- $(TRANSFORMDIR)/*_computed.csv
	rm -f -- $(TRANSFORMDIR)/*_diff

ffclean: fclean
	rm -f -- $(RUNTESTS)
	rm -f -- $(RANSAC)

re: fclean all

rere: ffclean all

.PHONY: all $(NAME) clean fclean ffclean re rere
