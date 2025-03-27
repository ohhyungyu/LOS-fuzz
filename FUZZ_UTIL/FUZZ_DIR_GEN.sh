echo "Insert fuzzing target package name"
read target_name

mkdir -pv $target_name/inputs \
    $target_name/outputs \
    $target_name/cvg_hour

# echo "Insert Random input seed count"
# read idx

head -c 50 /dev/random > $target_name/inputs/input0.txt