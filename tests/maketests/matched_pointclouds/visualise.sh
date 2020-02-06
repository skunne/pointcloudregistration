sed -e 's/,/ /g' $1 > tmp.txt
cut -d' ' -f1 --complement tmp.txt > tmp2.txt
python3 ../../../scripts_python/other/show_pcd_file.py tmp2.txt figure.png
rm -f tmp.txt tmp2.txt
