FUZZ_TARGET_DIR=FUZZ_TARGET

echo "Insert Download Package url(git)
EX) https://github.com/LOS/LOS"
read git_package_url

echo "Insert Option git command
EX) Insert : -b test
EX) Result : git clone ${git_package_url} ./TMP -b test"
read git_option

mkdir -pv TMP
git clone $git_package_url ./TMP $git_option

mv ./TMP/* $FUZZ_TARGET_DIR/
rm -rf TMP