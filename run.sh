BUILD_DIRECTORY=./cmake-build-debug
INSTANCES=$1
shift

echo "Solving using idol"
for FILE in $INSTANCES/*
do
  echo "$(date) ${FILE} ${@}"
  $BUILD_DIRECTORY/solve_kp $FILE $@
done
